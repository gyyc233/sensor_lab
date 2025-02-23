#include "eskf.hpp"
#include "navigation_and_mapping/io_utils.h"
#include "static_imu_init.h"
#include "utm_convert.h"

#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iomanip>
#include <iostream>

using namespace sad;

// gflags 处理命令行参数 DEFINE_string(变量名，默认值，描述);
DEFINE_string(txt_path, "./data/imu/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(with_odom, true, "是否加入轮速计信息");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (fLS::FLAGS_txt_path.empty()) {
    return -1;
  }

  // 初始化器
  sad::StaticIMUInit imu_init; // 使用默认配置
  sad::ESKFD eskf;

  sad::TxtIO io(FLAGS_txt_path);
  Vec2d antenna_pos(FLAGS_antenna_pox_x, FLAGS_antenna_pox_y);

  auto save_vec3 = [](std::ofstream &fout, const Vec3d &v) {
    fout << v[0] << " " << v[1] << " " << v[2] << " ";
  };
  auto save_quat = [](std::ofstream &fout, const Quatd &q) {
    fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
  };

  auto save_result = [&save_vec3,
                      &save_quat](std::ofstream &fout,
                                  const sad::NavStated &save_state) {
    fout << std::setprecision(18) << save_state.timestamp_ << " "
         << std::setprecision(9);
    save_vec3(fout, save_state.p_);                   // position
    save_quat(fout, save_state.R_.unit_quaternion()); // rotation quaternion
    save_vec3(fout, save_state.v_);                   // velocity
    save_vec3(fout, save_state.bg_);                  // gyro zero bias
    save_vec3(fout, save_state.ba_);                  // accelerate zero bias
    fout << std::endl;
  };

  std::ofstream fout("./data/eskf/gins.txt");
  bool imu_inited = false, gnss_inited = false;

  /// 设置各类回调函数
  bool first_gnss_set = false;
  Vec3d origin = Vec3d::Zero();

  // this 指针的链式调用
  // 1. call SetIMUProcessFunc
  // 2. call SetGNSSProcessFunc
  // 3. call SetOdomProcessFunc
  io.SetIMUProcessFunc([&](const sad::IMU &imu) {
      // imu process
      if (!imu_init.InitSuccess()) {
        imu_init.AddIMU(imu);
        return;
      }

      // need imu init
      if (!imu_inited) {
        // 读取初始零偏，设置ESKF
        sad::ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var = sqrt(imu_init.GetCovGyro()[0]);
        options.acce_var = sqrt(imu_init.GetCovAcce()[0]);
        eskf.SetInitialConditions(options, imu_init.GetInitBg(),
                                  imu_init.GetInitBa(), imu_init.GetGravity());
        imu_inited = true;
        return;
      }

      if (!gnss_inited) {
        /// 等待有效的RTK数据
        return;
      }

      /// GNSS 也接收到之后，再开始进行预测
      eskf.Predict(imu);

      /// predict就会更新ESKF，所以此时就可以发送数据
      auto state = eskf.GetNominalState();
    })
      .SetGNSSProcessFunc([&](const sad::GNSS &gnss) {
        /// GNSS 处理函数
        if (!imu_inited) {
          return;
        }

        sad::GNSS gnss_convert = gnss;
        if (!sad::ConvertGps2UTM(gnss_convert, antenna_pos,
                                 FLAGS_antenna_angle) ||
            !gnss_convert.heading_valid_) {
          return;
        }

        /// 去掉原点
        if (!first_gnss_set) {
          origin = gnss_convert.utm_pose_.translation();
          first_gnss_set = true;
        }
        gnss_convert.utm_pose_.translation() -= origin;

        // 要求RTK heading有效，才能合入ESKF
        eskf.ObserveGps(gnss_convert);

        auto state = eskf.GetNominalState();
        save_result(fout, state);
        gnss_inited = true;
      })
      .SetOdomProcessFunc([&](const sad::Odom &odom) {
        /// Odom 处理函数，本章Odom只给初始化使用
        imu_init.AddOdom(odom);
        if (FLAGS_with_odom && imu_inited && gnss_inited) {
          eskf.ObserveWheelSpeed(odom);
        }
      })
      .Go();
}
