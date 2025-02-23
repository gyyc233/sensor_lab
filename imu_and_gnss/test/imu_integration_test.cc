#include "navigation_and_mapping/io_utils.h"
#include "imu_integration.h"
#include <glog/logging.h>
#include <iomanip>

// imu 直接积分
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::GLOG_INFO);

  // assume imu bias
  Vec3d gravity(0, 0, -9.8);
  Vec3d init_bg(00.000224886, -7.61038e-05, -0.000742259);
  Vec3d init_ba(-0.165205, 0.0926887, 0.0058049);

  sad::IMUIntegration imu_integration(gravity, init_bg, init_ba);

  /// 记录结果
  auto save_result = [](std::ofstream &fout, double timestamp,
                        const Sophus::SO3 &R, const Vec3d &v, const Vec3d &p) {
    auto save_vec3 = [](std::ofstream &fout, const Vec3d &v) {
      fout << v[0] << " " << v[1] << " " << v[2] << " ";
    };
    auto save_quat = [](std::ofstream &fout, const Quatd &q) {
      fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };

    fout << std::setprecision(18) << timestamp << " " << std::setprecision(9);
    save_vec3(fout, p);
    save_quat(fout, R.unit_quaternion());
    save_vec3(fout, v);
    fout << std::endl;
  };

  sad::TxtIO io("./data/imu/10.txt");
  std::ofstream fout("./data/imu/imu_integration_state.txt");
  // [&imu_integration, &save_result, &fout] lambda 按引用捕获这三个变量
  io.SetIMUProcessFunc(
        [&imu_integration, &save_result, &fout](const sad::IMU &imu) {
          imu_integration.addIMU(imu);
          save_result(fout, imu.timestamp_, imu_integration.getR(),
                      imu_integration.getV(), imu_integration.getP());
        })
      .Go();

  return 0;
}
