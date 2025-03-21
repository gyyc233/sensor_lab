#ifdef ROS_CATKIN
#include "mapping_2d.h"
#include "lidar_2d_utils.h"
#include "submap.h"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
namespace sad {

bool Mapping2D::init(bool with_loop_closing) {
  keyframe_id_ = 0;
  // set origin submap
  current_submap_ = std::make_shared<Submap>(SE2());
  all_submaps_.emplace_back(current_submap_);

  return true;
}

bool Mapping2D::processScan(Scan2d::Ptr scan) {
  current_frame_ = std::make_shared<Frame>(scan);
  current_frame_->id_ = frame_id_++;
  LOG(INFO) << "process scan id: " << current_frame_->id_;

  // TODO:
  if (last_frame_) {
    // set pose from last frame
    current_frame_->pose_ = last_frame_->pose_ * motion_guess_;
    current_frame_->pose_submap_ = last_frame_->pose_submap_;
  }

  // 利用scan matching来匹配地图
  if (!first_scan_) {
    // 第一帧无法匹配，直接加入到occupancy map
    current_submap_->matchScan(current_frame_);
  }

  first_scan_ = false;
  // 判断当前帧是否为关键帧
  bool is_kf = isKeyFrame();

  if (is_kf) {
    // 添加关键帧并将其加入栅格
    addKeyFrame();
    current_submap_->addScanInOccupancyMap(current_frame_);

    // 处理回环检测
    // if (loop_closing_) {
    //     loop_closing_->AddNewFrame(current_frame_);
    // }

    if (current_submap_->hasOutsidePoints() ||
        (current_submap_->numFrames()) > 50) {
      /// 走出了submap或者单个submap中的关键帧较多8
      expandSubmap();
    }
  }

  // 可视化输出
  auto occu_image =
      current_submap_->getOccupancyMap().getOccupancyGridBlackWhite();
  Visualize2DScan(current_frame_->scan_, current_frame_->pose_, occu_image,
                  Vec3b(0, 0, 255), 1000, 20.0f, current_submap_->getPose());
  cv::putText(occu_image, "submap " + std::to_string(current_submap_->getId()),
              cv::Point2f(20, 20), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 255, 0));
  cv::putText(occu_image,
              "keyframes " + std::to_string(current_submap_->numFrames()),
              cv::Point2f(20, 50), cv::FONT_HERSHEY_COMPLEX, 0.5,
              cv::Scalar(0, 255, 0));
  // cv::imshow("occupancy map", occu_image);

  auto field_image = current_submap_->getLikelihood().getFieldImage();
  Visualize2DScan(current_frame_->scan_, current_frame_->pose_, field_image,
                  Vec3b(0, 0, 255), 1000, 20.0f, current_submap_->getPose());
  // cv::imshow("likelihood", field_image);

  // show global map
  if (is_kf) {
    //   cv::imshow("global map", showGlobalMap());
  }
  // cv::waitKey(10);

  if (last_frame_) {
    // calculate delta pose
    // 比较当前帧和上一帧
    motion_guess_ = last_frame_->pose_.inverse() * current_frame_->pose_;
  }

  last_frame_ = current_frame_;

  return true;
}

bool Mapping2D::isKeyFrame() {
  if (last_keyframe_ == nullptr) {
    return true;
  }

  // 比较当前帧和最近一帧关键帧
  SE2 delta_pose = last_keyframe_->pose_.inverse() * current_frame_->pose_;

  // 检查偏移量
  if (delta_pose.translation().norm() > keyframe_pos_th_ ||
      fabs(delta_pose.so2().log()) > keyframe_ang_th_) {
    return true;
  }

  return false;
}

void Mapping2D::addKeyFrame() {
  LOG(INFO) << "add keyframe " << keyframe_id_;
  current_frame_->keyframe_id_ = keyframe_id_++;
  current_submap_->addKeyFrame(current_frame_);
  last_keyframe_ = current_frame_;
}

void Mapping2D::expandSubmap() {
  // 当前submap作为历史地图放入loop closing
  // if (loop_closing_) {
  //     loop_closing_->AddFinishedSubmap(current_submap_);
  // }

  // 将当前submap替换成新的
  auto last_submap = current_submap_;

  // debug
  cv::imwrite("./data/ch6/submap_" + std::to_string(last_submap->getId()) +
                  ".png",
              last_submap->getOccupancyMap().getOccupancyGridBlackWhite());

  current_submap_ = std::make_shared<Submap>(current_frame_->pose_);
  current_frame_->pose_submap_ = SE2(); // 这个归零

  current_submap_->setId(++submap_id_);
  current_submap_->addKeyFrame(current_frame_);
  current_submap_->setOccuFromOtherSubmap(
      last_submap); // 把上一帧的部分数据也放进来，不让一个submap显得太空

  current_submap_->addScanInOccupancyMap(current_frame_);
  all_submaps_.emplace_back(current_submap_);

  // if (loop_closing_) {
  //     loop_closing_->addNewSubmap(current_submap_);
  // }

  LOG(INFO) << "create submap " << current_submap_->getId() << " with pose: "
            << current_submap_->getPose().translation().transpose() << ", "
            << current_submap_->getPose().so2().log();
}

cv::Mat Mapping2D::showGlobalMap(int max_size) {
  //// TODO 全局地图固定大小，使用动态分辨率
  Vec2d top_left = Vec2d(999999, 999999);
  Vec2d bottom_right = Vec2d(-999999, -999999);

  const double submap_resolution =
      20.0; // 子地图分辨率（1米多少个像素 pixel/m）
  const double submap_size = 50.0; // 单个submap大小

  /// 计算全局地图物理边界
  for (auto m : all_submaps_) {
    Vec2d c = m->getPose().translation();
    if (top_left[0] > c[0] - submap_size / 2) {
      top_left[0] = c[0] - submap_size / 2;
    }
    if (top_left[1] > c[1] - submap_size / 2) {
      top_left[1] = c[1] - submap_size / 2;
    }

    if (bottom_right[0] < c[0] + submap_size / 2) {
      bottom_right[0] = c[0] + submap_size / 2;
    }
    if (bottom_right[1] < c[1] + submap_size / 2) {
      bottom_right[1] = c[1] + submap_size / 2;
    }
  }

  if (top_left[0] > bottom_right[0] || top_left[1] > bottom_right[1]) {
    return cv::Mat();
  }

  /// 全局地图物理中心
  Vec2d global_center = Vec2d((top_left[0] + bottom_right[0]) / 2.0,
                              (top_left[1] + bottom_right[1]) / 2.0);
  float phy_width = bottom_right[0] - top_left[0];  // 物理尺寸
  float phy_height = bottom_right[1] - top_left[1]; // 物理尺寸
  double global_map_resolution = 0; // 1米占多少个像素 pixel/m）

  if (phy_width > phy_height) {
    global_map_resolution = max_size / phy_width;
  } else {
    global_map_resolution = max_size / phy_height;
  }

  Vec2d c = global_center;
  // 全局地图图像中心
  int c_x = global_center[0] * global_map_resolution;
  int c_y = global_center[1] * global_map_resolution;

  // 计算全局地图像素大小
  int width =
      int((bottom_right[0] - top_left[0]) * global_map_resolution + 0.5);
  int height =
      int((bottom_right[1] - top_left[1]) * global_map_resolution + 0.5);

  Vec2d center_image = Vec2d(width / 2, height / 2);
  // opencv: rows y 行 height,cols x 列 width
  cv::Mat output_image(height, width, CV_8UC3, cv::Scalar(127, 127, 127));

  // 开始渲染
  std::vector<Vec2i> render_data;
  render_data.reserve(width * height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      render_data.emplace_back(Vec2i(x, y));
    }
  }

  std::for_each(render_data.begin(), render_data.end(), [&](const Vec2i &xy) {
    int x = xy[0], y = xy[1]; // x--row,y--col
    // 预先计算每个像素坐标转世界坐标
    Vec2d pw = (Vec2d(x, y) - center_image) / global_map_resolution + c;

    for (auto &m : all_submaps_) {
      // TODO: check at first outside loop, pw was [0,0]
      Vec2d ps = m->getPose().inverse() * pw; // in submap
      // 转换为像素
      Vec2d pt = (ps * submap_resolution + Vec2d(500, 500));

      if (pt[0] < 0 || pt[0] >= 1000 || pt[1] < 0 || pt[1] >= 1000) {
        continue;
      }

      // 检查该子地图世界坐标对应像素的情况
      uchar value = m->getOccupancyMap().getOccupancyGrid().at<uchar>(
          static_cast<int>(pt[1]), static_cast<int>(pt[0]));
      if (value > 127) {
        if (m == current_submap_) {
          // 为当前子地图且remain
          output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(235, 250, 230);
        } else {
          output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
        }
        break;
      } else if (value < 127) {
        if (m == current_submap_) {
          // 为当前子地图且occupied
          output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(230, 20, 30);
        } else {
          output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
        }
        break;
      }
    }
  });

  for (auto &m : all_submaps_) {
    /// submap pose 在全局地图中的投影
    SE2 submap_pose = m->getPose();
    // get world pose
    Vec2d submap_center = submap_pose.translation();
    Vec2d submap_xw = submap_pose * Vec2d(1.0, 0);
    Vec2d submap_yw = submap_pose * Vec2d(0, 1.0);

    // world pose to pixel
    Vec2d center_map =
        (submap_center - global_center) * global_map_resolution + center_image;
    Vec2d x_map =
        (submap_xw - global_center) * global_map_resolution + center_image;
    Vec2d y_map =
        (submap_yw - global_center) * global_map_resolution + center_image;

    // x轴和y轴
    cv::line(output_image, cv::Point2f(center_map.x(), center_map.y()),
             cv::Point2f(x_map.x(), x_map.y()), cv::Scalar(0, 0, 255), 2);
    cv::line(output_image, cv::Point2f(center_map.x(), center_map.y()),
             cv::Point2f(y_map.x(), y_map.y()), cv::Scalar(0, 255, 0), 2);
    cv::putText(output_image, std::to_string(m->getId()),
                cv::Point2f(center_map.x() + 10, center_map.y() - 10),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0));

    // 子地图中所有关键帧的轨迹
    for (const auto &frame : m->getFrames()) {
      Vec2d p_map =
          (frame->pose_.translation() - global_center) * global_map_resolution +
          center_image;
      cv::circle(output_image, cv::Point2f(p_map.x(), p_map.y()), 1,
                 cv::Scalar(0, 0, 255), 1);
    }
  }

  // if (loop_closing_) {
  //   /// 回环检测的pose graph
  //   auto loops = loop_closing_->GetLoops();
  //   for (auto lc : loops) {
  //     auto first_id = lc.first.first;
  //     auto second_id = lc.first.second;

  //     Vec2f c1 =
  //         all_submaps_[first_id]->getPose().translation();
  //     Vec2f c2 =
  //         all_submaps_[second_id]->getPose().translation();

  //     Vec2f c1_map =
  //         (c1 - global_center) * global_map_resolution + center_image;
  //     Vec2f c2_map =
  //         (c2 - global_center) * global_map_resolution + center_image;

  //     cv::line(output_image, cv::Point2f(c1_map.x(), c1_map.y()),
  //              cv::Point2f(c2_map.x(), c2_map.y()), cv::Scalar(255, 0, 0),
  //              2);
  //   }
  // }

  return output_image;
}
} // namespace sad

#endif