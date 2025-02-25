#ifndef __IO_UTILS_H__
#define __IO_UTILS_H__

#include <fstream>
#include <functional>
#include <utility>

#include "../../imu_and_gnss/include/utm_convert.h"
#include "dataset_type.h" // 某些数据集会用到的类型
#include "gnss.h"
#include "imu.h"
#include "odom.h"

#ifdef ROS_CATKIN
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#endif

namespace sad {
class TxtIO {
public:
  TxtIO(const std::string &file_path) : fin(file_path) {}

  /// 定义回调函数
  using IMUProcessFuncType = std::function<void(const IMU &)>;
  using GNSSProcessFuncType = std::function<void(const GNSS &)>;
  using OdomProcessFuncType = std::function<void(const Odom &)>;

  // register imu callback
  TxtIO &SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
    imu_proc_ = std::move(imu_proc);
    return *this;
  }

  // register gnss callback
  TxtIO &SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
    gnss_proc_ = std::move(gnss_proc);
    return *this;
  }

  // register odom callback
  TxtIO &SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
    odom_proc_ = std::move(odom_proc);
    return *this;
  }

  // 遍历文件内容，调用回调函数
  void Go();

private:
  std::ifstream fin;
  IMUProcessFuncType imu_proc_;
  GNSSProcessFuncType gnss_proc_;
  OdomProcessFuncType odom_proc_;
};

#ifdef ROS_CATKIN

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
public:
  explicit RosbagIO(std::string bag_file,
                    DatasetType dataset_type = DatasetType::NCLT)
      : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
    assert(dataset_type_ != DatasetType::UNKNOWN);
  }

  using MessageProcessFunction =
      std::function<bool(const rosbag::MessageInstance &m)>;

  /// 一些方便直接使用的topics, messages
  using Scan2DHandle = std::function<bool(sensor_msgs::LaserScanPtr)>;
  // using MultiScan2DHandle = std::function<bool(MultiScan2d::Ptr)>;
  // using PointCloud2Handle =
  // std::function<bool(sensor_msgs::PointCloud2::Ptr)>; using
  // FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
  using ImuHandle = std::function<bool(IMUPtr)>;
  using GNSSHandle = std::function<bool(GNSSPtr)>;
  using OdomHandle = std::function<bool(const Odom &)>;

  // 遍历文件内容，调用回调函数
  void Go();

  /// 通用处理函数
  RosbagIO &AddHandle(const std::string &topic_name,
                      MessageProcessFunction func) {
    process_func_.emplace(topic_name, func);
    return *this;
  }

  /// 2D激光处理
  RosbagIO &AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
    return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
      auto msg = m.instantiate<sensor_msgs::LaserScan>();
      if (msg == nullptr) {
        return false;
      }
      return f(msg);
    });
  }

  /// 多回波2D激光处理
  // RosbagIO &AddMultiScan2DHandle(const std::string &topic_name,
  //                                MultiScan2DHandle f) {
  //   return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) ->
  //   bool {
  //     auto msg = m.instantiate<MultiScan2d>();
  //     if (msg == nullptr) {
  //       return false;
  //     }
  //     return f(msg);
  //   });
  // }

  // /// 根据数据集类型自动确定topic名称
  // RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f) {
  //   if (dataset_type_ == DatasetType::WXB_3D) {
  //     return *this;
  //   } else if (dataset_type_ == DatasetType::AVIA) {
  //     // AVIA 不能直接获取point cloud 2
  //     return *this;
  //   } else {
  //     return AddHandle(GetLidarTopicName(),
  //                      [f](const rosbag::MessageInstance &m) -> bool {
  //                        auto msg =
  //                        m.instantiate<sensor_msgs::PointCloud2>(); if (msg
  //                        == nullptr) {
  //                          return false;
  //                        }
  //                        return f(msg);
  //                      });
  //   }
  // }

  /// 根据数据集自动处理RTK消息
  // RosbagIO &AddAutoRTKHandle(GNSSHandle f) {
  //   if (dataset_type_ == DatasetType::NCLT) {
  //     return AddHandle(nclt_rtk_topic,
  //                      [f, this](const rosbag::MessageInstance &m) -> bool {
  //                        auto msg = m.instantiate<sensor_msgs::NavSatFix>();
  //                        if (msg == nullptr) {
  //                          return false;
  //                        }

  //                        GNSS* gnss_ptr = new GNSS(msg);
  //                        GNSSPtr gnss(gnss_ptr);
  //                        ConvertGps2UTMOnlyTrans(*gnss);
  //                        if (std::isnan(gnss->lat_lon_alt_[2])) {
  //                          // 貌似有Nan
  //                          return false;
  //                        }

  //                        return f(gnss);
  //                      });
  //   } else {
  //     // TODO 其他数据集的RTK转换关系
  //   }
  // }

  /// point cloud 2 的处理
  // RosbagIO &AddPointCloud2Handle(const std::string &topic_name,
  //                                PointCloud2Handle f) {
  //   return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) ->
  //   bool {
  //     auto msg = m.instantiate<sensor_msgs::PointCloud2>();
  //     if (msg == nullptr) {
  //       return false;
  //     }
  //     return f(msg);
  //   });
  // }

  // /// IMU
  // RosbagIO &AddImuHandle(ImuHandle f);

  // /// 清除现有的处理函数
  // void CleanProcessFunc() { process_func_.clear(); }

private:
  /// 根据设定的数据集名称获取雷达名
  // std::string GetLidarTopicName() const;

  /// 根据数据集名称确定IMU topic名称
  // std::string GetIMUTopicName() const;

  std::map<std::string, MessageProcessFunction> process_func_;
  std::string bag_file_;
  DatasetType dataset_type_;
};

#endif

} // namespace sad

#endif