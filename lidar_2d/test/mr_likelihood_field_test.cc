#include <fstream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "frame.h"
#include "lidar_2d_utils.h"
#include "multi_resolution_likelihood_field.h"

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::ifstream fin("./data/mapping_2d/loops.txt");
  int loop_id = 0;

  while (!fin.eof()) {
    int frame_id, submap_id;
    double submap_center_x, submap_center_y, theta;
    // peek预读并返回下一个字符 到达文件结束符(eof)
    if (fin.peek() == fin.eof()) {
      break;
    }

    fin >> frame_id >> submap_id >> submap_center_x >> submap_center_y >> theta;
    loop_id++;

    sad::MRLikelihoodField mr_field;
    Vec2d center(submap_center_x, submap_center_y);
    SE2 pose_submap(SO2::exp(theta), center);

    mr_field.setPose(pose_submap);
    cv::Mat occu_map = cv::imread("./data/mapping_2d/submap_" +
                                      std::to_string(submap_id) + ".png",
                                  cv::IMREAD_GRAYSCALE);
    cv::Mat occu_map_color = cv::imread("./data/mapping_2d/submap_" +
                                            std::to_string(submap_id) + ".png",
                                        cv::IMREAD_COLOR);
    mr_field.setFieldImageFromOccuMap(occu_map);

    sad::Frame frame;
    frame.load("./data/mapping_2d/frame_" + std::to_string(frame_id) + ".txt");
    mr_field.setSourceScan(frame.scan_);

    LOG(INFO) << "testing frame " << frame.id_ << " with " << submap_id;

    auto init_pose = frame.pose_;
    // 计算frame在子地图的位姿
    auto frame_pose_in_submap =
        pose_submap.inverse() * frame.pose_; // 去掉子地图的世界坐标位姿
    bool align_success = mr_field.alignG2O(frame_pose_in_submap);

    if (align_success) {
      frame.pose_ = pose_submap * frame_pose_in_submap;
      auto images = mr_field.getFieldImage();
      for (int i = 0; i < images.size(); ++i) {
        /// 初始pose 以红色显示
        sad::Visualize2DScan(frame.scan_, init_pose, images[i],
                             Vec3b(0, 0, 255), images[i].rows,
                             mr_field.setResolution(i), pose_submap);
        /// 配准后pose 以绿色显示
        sad::Visualize2DScan(frame.scan_, frame.pose_, images[i],
                             Vec3b(0, 255, 0), images[i].rows,
                             mr_field.setResolution(i), pose_submap);
        cv::imshow("level " + std::to_string(i), images[i]);
      }

      sad::Visualize2DScan(frame.scan_, init_pose, occu_map_color,
                           Vec3b(0, 0, 255), occu_map_color.rows, 20.0,
                           pose_submap);
      sad::Visualize2DScan(frame.scan_, frame.pose_, occu_map_color,
                           Vec3b(0, 255, 0), occu_map_color.rows, 20.0,
                           pose_submap);
      cv::imshow("occupancy", occu_map_color);
      cv::waitKey();
    }
  }
  return 0;
}
