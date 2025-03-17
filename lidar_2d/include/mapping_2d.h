#ifndef MAPPING_2D_H
#define MAPPING_2D_H
#ifdef ROS_CATKIN

#include "eigen_type/eigen_types.h"
#include "frame.h"
#include "navigation_and_mapping/lidar_utils.h"
#include "submap.h"

#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace sad {

// class
// 前置声明，告诉编译器，这个类在其它地方定义，真正使用类的定义的时候才进行编译
// 优点：避免#include, 避免修改一个被包含的头文件，导致整个文件重新编译
// 缺点：前置声明函数或模板有时会害得头文件开发者难以轻易变动其 API
// class Submap;
// class LoopClosing;

/**
 * 2D 激光建图的主要类
 */
class Mapping2D {
public:
  bool init(bool with_loop_closing = false);

  /// @brief 单回波的scan 建图
  /// @param scan
  /// @return
  bool processScan(Scan2d::Ptr scan);

  /// 多回波的scan
  /// 暂时没用到
  // bool processScan(MultiScan2d::Ptr scan);

  /// @brief 显示全局地图
  /// @param max_size 全局地图最大长宽(pixels)
  /// @return 全局地图图像
  cv::Mat showGlobalMap(int max_size = 500);

private:
  /// @brief 判定当前帧是否为关键帧
  /// @return
  bool isKeyFrame();

  /// @brief 增加一个关键帧
  /// @return
  void addKeyFrame();

  /// @brief 扩展新的submap
  /// @return
  void expandSubmap();

  /// 数据成员
  size_t frame_id_ = 0;    // 帧id
  size_t keyframe_id_ = 0; // 关键帧id
  size_t submap_id_ = 0;   // 子地图id

  bool first_scan_ = true;
  std::shared_ptr<Frame> current_frame_ = nullptr;
  std::shared_ptr<Frame> last_frame_ = nullptr;
  SE2 motion_guess_;
  std::shared_ptr<Frame> last_keyframe_ = nullptr;
  std::shared_ptr<Submap> current_submap_ = nullptr;

  std::vector<std::shared_ptr<Submap>> all_submaps_;

  // std::shared_ptr<LoopClosing> loop_closing_ = nullptr; // 回环检测

  // 参数
  inline static constexpr double keyframe_pos_th_ = 0.3; // 关键帧位移量
  inline static constexpr double keyframe_ang_th_ =
      15 * M_PI / 180; // 关键帧角度量
};
} // namespace sad

#endif
#endif
