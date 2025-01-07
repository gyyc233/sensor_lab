#include <glog/logging.h>
#include <iomanip>
#include <memory>

#include "common/gnss.h"
#include "common/io_utils.h"
#include "utm_convert.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::GLOG_INFO);

  std::string gnss_txt_path = "./data/imu/10.txt";
  std::vector<double> antenna = {
      12.06, -0.17,
      -0.20}; // RTK天线安装偏角(degree), RTK天线安装偏移X, RTK天线安装偏移Y
  Vec2d antenna_pos(-0.17, -0.2);

  // 记录gnss结果
  auto save_result = [](std::ofstream &fout, double timestamp,
                        const SE3 &pose) {
    // get translation part
    auto save_vec3 = [](std::ofstream &fout, const Vec3d &v) {
      fout << v[0] << " " << v[1] << " " << v[2] << " ";
    };

    // get rotation part
    auto save_quat = [](std::ofstream &fout, const Quatd &q) {
      fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);

    save_vec3(fout, pose.translation());
    save_quat(fout, pose.unit_quaternion());
    fout << std::endl;
  };

  sad::TxtIO io("./data/imu/10.txt"); // 这个文件里面也包含了gnss
  std::ofstream fout("./data/gnss/gnss_integration_state.txt");

  bool first_gnss_set = false;
  Vec3d origin = Vec3d::Zero();
  io.SetGNSSProcessFunc([&](const sad::GNSS &gnss) {
      sad::GNSS gnss_out = gnss;

      if (sad::ConvertGps2UTM(gnss_out, antenna_pos, antenna[0])) {
        if (!first_gnss_set) {
          origin = gnss_out.utm_pose_.translation();
          first_gnss_set = true;
        }

        // subtract an origin pose
        gnss_out.utm_pose_.translation() -= origin;

        save_result(fout, gnss_out.unix_time_, gnss_out.utm_pose_);
      }
    })
      .Go();
  return 0;
}
