#include "gen_simu_data.h"
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv) {
  sad::GenSimuData gen_simu_data;
  gen_simu_data.genData();

  pcl::io::savePCDFileASCII("./data/mapping_3d/sim_target.pcd",
                            *gen_simu_data.getTarget());
  pcl::io::savePCDFileASCII("./data/mapping_3d/sim_source.pcd",
                            *gen_simu_data.getSource());

  // source = pose * target
  SE3 T_target_source = gen_simu_data.getPose().inverse();
  LOG(INFO) << "gt pose: " << T_target_source.translation().transpose() << ", "
            << T_target_source.so3().unit_quaternion().coeffs().transpose();

  return 0;
}
