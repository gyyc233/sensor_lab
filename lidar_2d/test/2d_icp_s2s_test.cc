#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "icp_2d.h"
#include "lidar_2d_utils.h"
#include "navigation_and_mapping/io_utils.h"

DEFINE_string(bag_path, "./data/2dmapping/floor2.bag", "数据包路径");
DEFINE_string(method, "point2point", "2d icp方法: point2point/point2plane");

/// 测试从rosbag中读取2d scan并plot的结果
/// 通过选择method来确定使用点到点或点到面的ICP
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
  Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

  /// 我们将上一个scan与当前scan进行配准
  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [&](Scan2d::Ptr scan) {
                         current_scan = scan;

                         if (last_scan == nullptr) {
                           last_scan = current_scan;
                           return true;
                         }

                         sad::Icp2d icp;
                         icp.setTarget(last_scan);
                         icp.setSource(current_scan);

                         SE2 pose;
                         if (fLS::FLAGS_method == "point2point") {
                           icp.alignGaussNewton(pose);
                         } else if (fLS::FLAGS_method == "point2plane") {
                           icp.alignGaussNewtonPoint2Plane(pose);
                         }

                         cv::Mat image;
                         sad::Visualize2DScan(last_scan, SE2(), image,
                                              Vec3b(255, 0, 0)); // target是蓝的
                         sad::Visualize2DScan(current_scan, pose, image,
                                              Vec3b(0, 0, 255)); // source是红的
                         cv::imshow("scan", image);
                         cv::waitKey(20);

                         last_scan = current_scan;
                         return true;
                       })
      .Go();

  return 0;
}
