#include "based_imu_pre_integration_gins.h"
#include "g2o_preintegration_types.h"
#include "g2o_types/g2o_types.h"
#include "navigation_and_mapping/io_utils.h"
#include "static_imu_init.h"
#include "utm_convert.h"

#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iomanip>

DEFINE_string(txt_path, "./data/imu/10.txt", "数据文件路径");
DEFINE_double(antenna_angle, 12.06, "RTK天线安装偏角（角度）");
DEFINE_double(antenna_pox_x, -0.17, "RTK天线安装偏移X");
DEFINE_double(antenna_pox_y, -0.20, "RTK天线安装偏移Y");
DEFINE_bool(debug, true, "是否打印调试信息");

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

  sad::TxtIO io(fLS::FLAGS_txt_path);
  Vec2d antenna_pos(fLD::FLAGS_antenna_pox_x, fLD::FLAGS_antenna_pox_y);

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
    save_vec3(fout, save_state.p_);
    save_quat(fout, save_state.R_.unit_quaternion());
    save_vec3(fout, save_state.v_);
    save_vec3(fout, save_state.bg_);
    save_vec3(fout, save_state.ba_);
    fout << std::endl;
  };

  std::ofstream fout("./data/eskf/gins_preintg.txt");
  bool imu_inited = false, gnss_inited = false;

  sad::GinsImuPreIntegration::Options gins_options;
  gins_options.verbose = FLAGS_debug;
  sad::GinsImuPreIntegration gins(gins_options);

  bool first_gnss_set = false;
  Vec3d origin = Vec3d::Zero();

  /// 设置各类回调函数
  io.SetIMUProcessFunc([&](const sad::IMU &imu) {
      /// IMU 处理函数
      if (!imu_init.InitSuccess()) {
        imu_init.AddIMU(imu);
        return;
      }

      /// 需要IMU初始化
      if (!imu_inited) {
        // 读取初始零偏，设置GINS
        // 使用静止初始化得到的零偏和重力 来初始化预计分类的零偏和重力
        sad::GinsImuPreIntegration::Options options;
        options.preinteg_options.init_bg_ = imu_init.GetInitBg();
        options.preinteg_options.init_ba_ = imu_init.GetInitBa();
        options.gravity = imu_init.GetGravity();

        // 设置信息矩阵与先验因子
        gins.SetOptions(options);
        imu_inited = true;
        return;
      }

      if (!gnss_inited) {
        /// 等待有效的RTK数据
        return;
      }

      /// GNSS 也接收到之后，再开始进行预测
      /// 初始化完成后接受imu data
      gins.AddImu(imu);

      // 预测
      auto state = gins.GetState();
      save_result(fout, state);
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

        gins.AddGnss(gnss_convert);

        auto state = gins.GetState();
        save_result(fout, state);
        gnss_inited = true;
      })
      .SetOdomProcessFunc([&](const sad::Odom &odom) {
        imu_init.AddOdom(odom);

        if (imu_inited && gnss_inited) {
          gins.AddOdom(odom);
        }
      })
      .Go();

  return 0;
}
