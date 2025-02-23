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

#ifdef ROS_CATKIN

void RosbagIO::Go() {
  rosbag::Bag bag(bag_file_);
  LOG(INFO) << "running in " << bag_file_
            << ", reg process func: " << process_func_.size();

  if (!bag.isOpen()) {
    LOG(ERROR) << "cannot open " << bag_file_;
    return;
  }

  auto view = rosbag::View(bag);
  for (const rosbag::MessageInstance &m : view) {
    auto iter = process_func_.find(m.getTopic());
    if (iter != process_func_.end()) {
      iter->second(m);
    }

    if (global::FLAG_EXIT) {
      break;
    }
  }

  bag.close();
  LOG(INFO) << "bag " << bag_file_ << " finished.";
}

RosbagIO &RosbagIO::AddImuHandle(RosbagIO::ImuHandle f) {
  return AddHandle(
      GetIMUTopicName(), [&f, this](const rosbag::MessageInstance &m) -> bool {
        auto msg = m.template instantiate<sensor_msgs::Imu>();
        if (msg == nullptr) {
          return false;
        }

        IMUPtr imu;
        if (dataset_type_ == DatasetType::AVIA) {
          // Livox内置imu的加计需要乘上重力常数
          imu = std::make_shared<IMU>(
              msg->header.stamp.toSec(),
              Vec3d(msg->angular_velocity.x, msg->angular_velocity.y,
                    msg->angular_velocity.z),
              Vec3d(msg->linear_acceleration.x * 9.80665,
                    msg->linear_acceleration.y * 9.80665,
                    msg->linear_acceleration.z * 9.80665));
        } else {
          imu = std::make_shared<IMU>(
              msg->header.stamp.toSec(),
              Vec3d(msg->angular_velocity.x, msg->angular_velocity.y,
                    msg->angular_velocity.z),
              Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                    msg->linear_acceleration.z));
        }
        return f(imu);
      });
}

std::string RosbagIO::GetIMUTopicName() const {
  if (dataset_type_ == DatasetType::ULHK) {
    return ulhk_imu_topic;
  } else if (dataset_type_ == DatasetType::UTBM) {
    return utbm_imu_topic;
  } else if (dataset_type_ == DatasetType::NCLT) {
    return nclt_imu_topic;
  } else if (dataset_type_ == DatasetType::WXB_3D) {
    return wxb_imu_topic;
  } else if (dataset_type_ == DatasetType::AVIA) {
    return avia_imu_topic;
  } else {
    LOG(ERROR) << "cannot load imu topic name of dataset "
               << int(dataset_type_);
  }

  return "";
}

#endif

} // namespace sad
