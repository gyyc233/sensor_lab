#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "lidar_2d_utils.h"
#include "navigation_and_mapping/io_utils.h"
#include "occupancy_map.h"
#include "sys_utils.h"

DEFINE_string(bag_path, "./data/2dmapping/floor2.bag", "数据包路径");
DEFINE_string(method, "model/bresenham", "填充算法：model/bresenham");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);

  /// 测试单个scan生成出来的occupancy grid是否正确
  rosbag_io
      .AddScan2DHandle(
          "/pavo_scan_bottom",
          [&](Scan2d::Ptr scan) {
            sad::OccupancyMap oc_map;
            if (FLAGS_method == "model") {
              sad::evaluate_and_call(
                  [&]() {
                    oc_map.addLidarFrame(
                        std::make_shared<sad::Frame>(scan),
                        sad::OccupancyMap::GridMethod::MODEL_POINTS);
                  },
                  "Occupancy with model points", 1);
            } else {
              sad::evaluate_and_call(
                  [&]() {
                    oc_map.addLidarFrame(
                        std::make_shared<sad::Frame>(scan),
                        sad::OccupancyMap::GridMethod::BRESENHAM);
                  },
                  "Occupancy with bresenham", 1);
            }
            cv::imshow("occupancy map", oc_map.getOccupancyGridBlackWhite());
            cv::waitKey(10);
            return true;
          })
      .Go();

  return 0;
}
