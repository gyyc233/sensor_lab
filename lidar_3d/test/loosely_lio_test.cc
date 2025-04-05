#include <gflags/gflags.h>
#include <glog/logging.h>

#include "loosely_coupled_lio/loosely_lio.h"
#include "navigation_and_mapping/io_utils.h"
#include "sys_utils.h"

DEFINE_string(bag_path, "./data/mapping_3d/ulhk_test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D"); // 数据集类型
DEFINE_string(config, "./data/mapping_3d/velodyne_ulhk.yaml",
              "path of config yaml"); // 配置文件类型
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path,
                          sad::Str2DatasetType(FLAGS_dataset_type));

  sad::LooselyLIO::Options options;
  options.with_ui_ = FLAGS_display_map;
  sad::LooselyLIO lm(options);
  lm.init(FLAGS_config);

  rosbag_io
      .AddAutoPointCloudHandle(
          [&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lm.PCLCallBack(cloud);
            return true;
          })
      .AddImuHandle([&](IMUPtr imu) {
        lm.IMUCallBack(imu);
        return true;
      })
      .Go();

  lm.finish();
  LOG(INFO) << "done.";

  return 0;
}
