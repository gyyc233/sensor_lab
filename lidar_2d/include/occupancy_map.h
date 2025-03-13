#ifndef LIDAR_2D_OCCUPANCY_MAP_H
#define LIDAR_2D_OCCUPANCY_MAP_H
#ifdef ROS_CATKIN

#include <opencv2/core.hpp>

#include "frame.h"

namespace sad {

class OccupancyMap {
public:
  /// 栅格模板，预先计算
  struct Model2DPoint {
    int dx_ = 0;
    int dy_ = 0;
    double angle_ = 0; // in rad
    float range_ = 0;  // in meters
  };

  enum class GridMethod {
    MODEL_POINTS, // 模板化算法
    BRESENHAM,    // 直接栅格化算法
  };

  OccupancyMap();

  /// @brief 往这个占据栅格地图中增加一个frame
  /// @param frame
  /// @param method
  void addLidarFrame(std::shared_ptr<Frame> frame,
                     GridMethod method = GridMethod::BRESENHAM);

  /// @brief 获取原始的占据栅格地图
  /// @return
  cv::Mat getOccupancyGrid() const { return occupancy_grid_; }

  /// @brief 获取黑白灰形式的占据栅格，作可视化使用
  /// @return
  cv::Mat getOccupancyGridBlackWhite() const;

  /// @brief 设置中心点
  /// @param pose
  void setPose(const SE2 &pose) { pose_ = pose; }

  /// @brief 是否有界外点
  /// @return
  bool hasOutsidePoints() const { return has_outside_pts_; }

  /// @brief 获取分辨率
  /// @return
  double resolution() const { return resolution_; }

  /// @brief 在某个点填入占据或者非占据信息
  /// @param pt
  /// @param occupy
  void setPoint(const Vec2i &pt, bool occupy);

private:
  /// @brief 生成填充模板
  void buildModel();

  /// @brief 从世界坐标系转到图像坐标系
  /// @tparam T
  /// @param pt
  /// @return
  template <class T>
  inline Vec2i world2Image(const Eigen::Matrix<T, 2, 1> &pt) {
    Vec2d pt_map = (pose_.inverse() * pt) * resolution_ + center_image_;
    int x = int(pt_map[0]);
    int y = int(pt_map[1]);
    return Vec2i(x, y);
  }

  /// @brief 查找某个角度下的range值
  /// @param angle
  /// @param scan
  /// @return
  double findRangeInAngle(double angle, Scan2d::Ptr scan);

  /// @brief Bresenham直线填充，给定起始点和终止点，将中间的区域填充为白色
  /// @param p1
  /// @param p2
  void bresenhamFilling(const Vec2i &p1, const Vec2i &p2);

  cv::Mat occupancy_grid_; // 8bit 占据栅格图像

  SE2 pose_; // T_W_S
  Vec2d center_image_ = Vec2d(image_size_ / 2, image_size_ / 2);

  bool has_outside_pts_ = false; // 标注栅格化过程中是否有落在外部的点

  // 模板
  std::vector<Model2DPoint> model_; // 用于填充占据栅格的模板，都是世界系下的点

  // 参数
  inline static constexpr double closest_th_ = 0.2; // 近距离阈值
  inline static constexpr double endpoint_close_th_ =
      0.1; // 末端点障碍物近距离阈值
  inline static constexpr double resolution_ = 20.0; // 1m 多少像素
  inline static constexpr float inv_resolution_ =
      0.05; // 1个像素多少米（栅格分辨率）
  inline static constexpr int image_size_ = 1000; // 图像大小
  inline static constexpr int model_size_ = 400;  // 模板像素大小
};

} // namespace sad

#endif
#endif
