#include <gflags/gflags.h>
#include <glog/logging.h>

#include "direct_ndt_lo.h"
#include "navigation_and_mapping/dataset_type.h"
#include "navigation_and_mapping/io_utils.h"
#include "ndt_3d.h"
#include "sys_utils.h"

/// 本程序以ULHK数据集为例
/// 测试以NDT为主的Lidar Odometry
/// 若使用PCL NDT的话，会重新建立NDT树
DEFINE_string(bag_path, "./data/mapping_3d/ulhk_test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D"); // 数据集类型
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path,
                          sad::Str2DatasetType(FLAGS_dataset_type));

  sad::Direct_NDT_LO::Options options;
  options.use_pcl_ndt_ = fLB::FLAGS_use_pcl_ndt;
  options.ndt3d_options_.nearby_type_ = FLAGS_use_ndt_nearby_6
                                            ? sad::NDT_3D::NearbyType::NEARBY6
                                            : sad::NDT_3D::NearbyType::CENTER;
  options.display_realtime_cloud_ = FLAGS_display_map;
  sad::Direct_NDT_LO ndt_lo(options);

  rosbag_io
      .AddAutoPointCloudHandle([&ndt_lo](
                                   sensor_msgs::PointCloud2::Ptr msg) -> bool {
        sad::evaluate_and_call(
            [&]() {
              SE3 pose;
              ndt_lo.addCloud(
                  sad::VoxelCloud(sad::PointCloud2ToCloudPtr(msg), 0.3), pose);
            },
            "NDT registration", 1);
        return true;
      })
      .Go();

  if (FLAGS_display_map) {
    // 把地图存下来
    ndt_lo.saveMap("./data/mapping_3d/map.pcd");
  }

  LOG(INFO) << "done.";

  return 0;
}
