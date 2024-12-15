#ifndef __IO_UTILS_H__
#define __IO_UTILS_H__

#include <fstream>
#include <functional>
#include <utility>

#include "common/imu.h"

namespace sad {
class TxtIO {
public:
  TxtIO(const std::string &file_path) : fin(file_path) {}

  /// 定义回调函数
  using IMUProcessFuncType = std::function<void(const IMU &)>;

  TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
    imu_proc_ = std::move(imu_proc);
    return *this;
  }

  // 遍历文件内容，调用回调函数
  void Go();

private:
  std::ifstream fin;
  IMUProcessFuncType imu_proc_;
};
} // namespace sad

#endif