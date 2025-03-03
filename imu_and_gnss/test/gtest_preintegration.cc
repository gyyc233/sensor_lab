#include "based_imu_pre_integration_gins.h"
#include "g2o_preintegration_types.h"
#include "g2o_types/g2o_types.h"
#include "imu_preintegration.h"
#include "navigation_and_mapping/io_utils.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

TEST(PREINTEGRATION_TEST, ROTATION_TEST) {
  // 测试在恒定角速度运转下的预积分情况
  double imu_time_span = 0.01; // IMU测量间隔 10ms
  Vec3d constant_omega(0, 0, M_PI); // 角速度为180度/s，转1秒应该等于转180度
  Vec3d gravity(0, 0, -9.8); // Z 向上，重力方向为负

  sad::NavStated start_status(0), end_status(1.0);
  sad::IMUPreintegration pre_integ;

  // 对比直接积分
  Sophus::SO3 R;
  Vec3d t = Vec3d::Zero();
  Vec3d v = Vec3d::Zero();

  for (int i = 1; i <= 100; ++i) {
    double time = imu_time_span * i;
    Vec3d acce = -gravity; // 加速度计应该测量到一个向上的力
    pre_integ.Integrate(sad::IMU(time, constant_omega, acce), imu_time_span);

    sad::NavStated this_status = pre_integ.Predict(start_status, gravity);

    // 直接积分
    t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
        0.5 * (R * acce) * imu_time_span * imu_time_span;
    v = v + gravity * imu_time_span + (R * acce) * imu_time_span;
    R = R * Sophus::SO3::exp(constant_omega * imu_time_span);

    // 验证在简单情况下，直接积分和预积分结果相等
    EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
    EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
    EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

    EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
    EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
    EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

    EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(),
                1e-4);
  }

  end_status = pre_integ.Predict(start_status);

  LOG(INFO) << "preinteg result: ";
  LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
  LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
  LOG(INFO) << "end v: \n" << end_status.v_.transpose();

  LOG(INFO) << "direct integ result: ";
  LOG(INFO) << "end rotation: \n" << R.matrix();
  LOG(INFO) << "end trans: \n" << t.transpose();
  LOG(INFO) << "end v: \n" << v.transpose();
  SUCCEED();
}

TEST(PREINTEGRATION_TEST, ACCELERATION_TEST) {
  // 测试在恒定加速度运行下的预积分情况
  double imu_time_span = 0.01;    // IMU测量间隔 10ms
  Vec3d gravity(0, 0, -9.8);      // Z 向上，重力方向为负
  Vec3d constant_acce(0.1, 0, 0); // x 方向上的恒定加速度

  sad::NavStated start_status(0), end_status(1.0);
  sad::IMUPreintegration pre_integ;

  // 对比直接积分
  Sophus::SO3 R;
  Vec3d t = Vec3d::Zero();
  Vec3d v = Vec3d::Zero();

  for (int i = 1; i <= 100; ++i) {
    double time = imu_time_span * i;
    Vec3d acce = constant_acce - gravity;
    pre_integ.Integrate(sad::IMU(time, Vec3d::Zero(), acce), imu_time_span);
    sad::NavStated this_status = pre_integ.Predict(start_status, gravity);

    t = t + v * imu_time_span + 0.5 * gravity * imu_time_span * imu_time_span +
        0.5 * (R * acce) * imu_time_span * imu_time_span;
    v = v + gravity * imu_time_span + (R * acce) * imu_time_span;

    // 验证在简单情况下，直接积分和预积分结果相等
    EXPECT_NEAR(t[0], this_status.p_[0], 1e-2);
    EXPECT_NEAR(t[1], this_status.p_[1], 1e-2);
    EXPECT_NEAR(t[2], this_status.p_[2], 1e-2);

    EXPECT_NEAR(v[0], this_status.v_[0], 1e-2);
    EXPECT_NEAR(v[1], this_status.v_[1], 1e-2);
    EXPECT_NEAR(v[2], this_status.v_[2], 1e-2);

    EXPECT_NEAR(R.unit_quaternion().x(), this_status.R_.unit_quaternion().x(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().y(), this_status.R_.unit_quaternion().y(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().z(), this_status.R_.unit_quaternion().z(),
                1e-4);
    EXPECT_NEAR(R.unit_quaternion().w(), this_status.R_.unit_quaternion().w(),
                1e-4);
  }

  end_status = pre_integ.Predict(start_status);
  LOG(INFO) << "preinteg result: ";
  LOG(INFO) << "end rotation: \n" << end_status.R_.matrix();
  LOG(INFO) << "end trans: \n" << end_status.p_.transpose();
  LOG(INFO) << "end v: \n" << end_status.v_.transpose();

  LOG(INFO) << "direct integ result: ";
  LOG(INFO) << "end rotation: \n" << R.matrix();
  LOG(INFO) << "end trans: \n" << t.transpose();
  LOG(INFO) << "end v: \n" << v.transpose();
  SUCCEED();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}
