#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "lidar_2d_utils.h"
#include "likelihood_field.h"
#include "navigation_and_mapping/io_utils.h"

DEFINE_string(bag_path, "./data/2dmapping/floor2.bag", "数据包路径");
DEFINE_string(method, "gauss-newton", "gauss-newton/g2o");

// 2D似然场法的ICP

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
  Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

  // 将上一个scan与当前scan进行配准
  rosbag_io
      .AddScan2DHandle("/pavo_scan_bottom",
                       [&](Scan2d::Ptr scan) {
                         sad::LikelihoodField lf;
                         current_scan = scan;
                         SE2 pose;

                         if (last_scan == nullptr) {
                           last_scan = current_scan;
                           return true;
                         }

                         lf.setTargetScan(last_scan);
                         lf.setSourceScan(current_scan);

                         if (FLAGS_method == "gauss-newton") {
                           lf.alignGaussNewton(pose);
                         } else if (FLAGS_method == "g2o") {
                           lf.alignG2O(pose);
                           return true;
                         }

                         LOG(INFO) << "aligned pose: "
                                   << pose.translation().transpose() << ", "
                                   << pose.so2().log();

                         cv::Mat image;
                         sad::Visualize2DScan(last_scan, SE2(), image,
                                              Vec3b(255, 0, 0)); // target是蓝的
                         sad::Visualize2DScan(current_scan, pose, image,
                                              Vec3b(0, 0, 255)); // source是红的
                         cv::imshow("scan", image);

                         /// 画出target和它的场函数
                         cv::Mat field_image = lf.getFieldImage();
                         sad::Visualize2DScan(last_scan, SE2(), field_image,
                                              Vec3b(255, 0, 0), 1000,
                                              20.0); // target是蓝的
                         cv::imshow("field", field_image);
                         cv::waitKey(10);

                         last_scan = current_scan;
                         return true;
                       })
      .Go();

  return 0;
}
