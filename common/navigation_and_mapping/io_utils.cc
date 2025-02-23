#include "io_utils.h"
#include <glog/logging.h>

namespace sad {
void TxtIO::Go() {
  if (!fin) {
    LOG(ERROR) << "未能找到文件";
    return;
  }

  while (!fin.eof()) {
    std::string line;
    std::getline(fin, line);
    if (line.empty()) {
      continue;
    }

    if (line[0] == '#') {
      // 以#开头的是注释
      continue;
    }

    // load data from line
    std::stringstream ss;
    ss << line;
    std::string data_type;
    ss >> data_type;

    if (data_type == "IMU" && imu_proc_) {
      double time, gx, gy, gz, ax, ay, az;
      ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
      // imu_proc_(IMU(time, Vec3d(gx, gy, gz) * math::kDEG2RAD, Vec3d(ax, ay,
      // az)));
      imu_proc_(IMU(time, Vec3d(gx, gy, gz), Vec3d(ax, ay, az)));
    } else if (data_type == "GNSS" && gnss_proc_) {
      // longitude 经度; latitude 纬度; altitude　高度
      double time, longitude, latitude, altitude, heading;
      bool heading_valid;
      // 数据以纬经高保存 emm...
      ss >> time >> latitude >> longitude >> altitude >> heading >>
          heading_valid;
      gnss_proc_(GNSS(time, 4, Vec3d(latitude, longitude, altitude), heading,
                      heading_valid));
    } else if (data_type == "ODOM" && odom_proc_) {
      double time, wl, wr;
      ss >> time >> wl >> wr;
      odom_proc_(Odom(time, wl, wr));
    }
  }

  LOG(INFO) << "done.";
}

} // namespace sad
