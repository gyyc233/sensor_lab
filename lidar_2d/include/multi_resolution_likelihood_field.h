#ifndef LIDAR_2D_MULTI_RESOLUTION_LIKELIHOOD_FILED_H
#define LIDAR_2D_MULTI_RESOLUTION_LIKELIHOOD_FILED_H
#ifdef ROS_CATKIN

#include <opencv2/core.hpp>

#include "eigen_type/eigen_types.h"
#include "navigation_and_mapping/lidar_utils.h"

namespace sad {
/// @brief 多分辨率的似然场陪准方法
class MRLikelihoodField {
  /// 2D 场的模板，在设置target scan或map的时候生成
  struct ModelPoint {
    ModelPoint(int dx, int dy, float res) : dx_(dx), dy_(dy), residual_(res) {}
    int dx_ = 0;
    int dy_ = 0;
    float residual_ = 0;
  };

  MRLikelihoodField() { buildModel(); }

  /// @brief 从占据栅格地图生成一个似然场地图
  /// @param occu_map
  void setFieldImageFromOccuMap(const cv::Mat &occu_map);

  /// @brief 使用g2o配准，内部调用不同层级的图像进行配准
  /// @param init_pose 待优化的位姿，也用于提供初始值
  /// @return
  bool alignG2O(SE2 &init_pose);

  /// @brief 获取场函数，转换为RGB图像
  /// @return
  std::vector<cv::Mat> getFieldImage();

  /// @brief 设置中心（通常即submap中心）
  /// @param pose
  void setPose(const SE2 &pose) { pose_ = pose; }

  /// @brief 设置匹配源
  /// @param scan
  void setSourceScan(Scan2d::Ptr scan) { source_ = scan; }

  /// @brief 设置栅格地图分辨率
  /// @param level
  /// @return
  float setResolution(int level = 0) const { return resolution_[level]; }

  /// @brief 图像金字塔层数
  /// @return
  int levels() const { return levels_; }

private:
  /**
   * 在某一层图像中配准
   * @param level
   * @param init_pose
   * @return
   */
  bool alignInLevel(int level, SE2 &init_pose);

  void buildModel();

  SE2 poses;

  Scan2d::Ptr source_ = nullptr;

  std::vector<ModelPoint> model_; // 2D 模板
  std::vector<cv::Mat> field_;    // 金字塔似然场函数

  std::vector<int> num_inliers_;     // 不同金字塔中，匹配后点数量
  std::vector<double> inlier_ratio_; // 不同金字塔中，匹配后的点比例

  // 金字塔层数
  inline static const int levels_ = 4;

  // 每米多少个像素
  inline static const std::vector<float> resolution_ = {2.5, 5, 10, 20};

  // 相比占据栅格图的比例尺
  inline static const std::vector<float> ratios_ = {0.125, 0.25, 0.5, 1.0};
};
} // namespace sad

#endif
#endif
