#include "loam_like_odom.h"
#include "navigation_and_mapping/io_utils.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(bag_path, "./data/mapping_3d/wxb/test1.bag", "path to wxb bag");
DEFINE_string(topic, "/velodyne_packets_1", "topic of lidar packets");
DEFINE_bool(display_map, true, "display map?");

using namespace sad;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::LoamLikeOdom::Options options;
  options.display_realtime_cloud_ = FLAGS_display_map;
  sad::LoamLikeOdom lo(options);

  LOG(INFO) << "using topic: " << FLAGS_topic;
  sad::RosbagIO bag_io(fLS::FLAGS_bag_path);
  // bag_io
  //     .AddVelodyneHandle(FLAGS_topic,
  //                        [&](sad::FullCloudPtr cloud) -> bool {
  //                          sad::common::Timer::Evaluate(
  //                              [&]() { lo.processPointCloud(cloud); },
  //                              "Loam-like odom");
  //                          return true;
  //                        })
  //     .Go();

  lo.SaveMap("./data/ch7/loam_map.pcd");

  return 0;
}
