#ifdef ROS_CATKIN

#include "feature_extraction.h"
#include "glog/logging.h"
#include <memory>

namespace sad {
FeatureExtraction::FeatureExtraction() {}

FeatureExtraction::~FeatureExtraction() {}

void FeatureExtraction::extract(FullCloudPtr pc_in, CloudPtr &pc_out_edge,
                                CloudPtr &pc_out_surf) {
  int num_scans = 16; // 线束(探头)数量,每个scan中包含16条线
  std::vector<CloudPtr> scans_in_each_line; // 保存线束
  for (int i = 0; i < num_scans; i++) {
    scans_in_each_line.emplace_back(new PointCloudType);
  }

  // 按照线束id进行分类
  for (auto &pt : pc_in->points) {
    assert(pt.ring >= 0 && pt.ring < num_scans);

    PointType p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    p.intensity = pt.intensity;

    scans_in_each_line[pt.ring]->points.emplace_back(p);
  }

  for (int i = 0; i < num_scans; i++) {
    if (scans_in_each_line[i]->points.size() < 131) {
      continue;
    }

    std::vector<IdAndValue> cloud_curvature;
    int ignore_edge_point_number = 5; // 忽视两侧的边缘点数量,这里表示一侧

    // 统计一条线
    for (int j = ignore_edge_point_number;
         j <
         (int)scans_in_each_line[i]->points.size() - ignore_edge_point_number;
         j++) {
      // 对每条线，统计每个点以及附近10个点的平均距离之和
      double sum_diffX = scans_in_each_line[i]->points[j - 5].x +
                         scans_in_each_line[i]->points[j - 4].x +
                         scans_in_each_line[i]->points[j - 3].x +
                         scans_in_each_line[i]->points[j - 2].x +
                         scans_in_each_line[i]->points[j - 1].x -
                         10 * scans_in_each_line[i]->points[j].x +
                         scans_in_each_line[i]->points[j + 1].x +
                         scans_in_each_line[i]->points[j + 2].x +
                         scans_in_each_line[i]->points[j + 3].x +
                         scans_in_each_line[i]->points[j + 4].x +
                         scans_in_each_line[i]->points[j + 5].x;
      double sum_diffY = scans_in_each_line[i]->points[j - 5].y +
                         scans_in_each_line[i]->points[j - 4].y +
                         scans_in_each_line[i]->points[j - 3].y +
                         scans_in_each_line[i]->points[j - 2].y +
                         scans_in_each_line[i]->points[j - 1].y -
                         10 * scans_in_each_line[i]->points[j].y +
                         scans_in_each_line[i]->points[j + 1].y +
                         scans_in_each_line[i]->points[j + 2].y +
                         scans_in_each_line[i]->points[j + 3].y +
                         scans_in_each_line[i]->points[j + 4].y +
                         scans_in_each_line[i]->points[j + 5].y;
      double sum_diffZ = scans_in_each_line[i]->points[j - 5].z +
                         scans_in_each_line[i]->points[j - 4].z +
                         scans_in_each_line[i]->points[j - 3].z +
                         scans_in_each_line[i]->points[j - 2].z +
                         scans_in_each_line[i]->points[j - 1].z -
                         10 * scans_in_each_line[i]->points[j].z +
                         scans_in_each_line[i]->points[j + 1].z +
                         scans_in_each_line[i]->points[j + 2].z +
                         scans_in_each_line[i]->points[j + 3].z +
                         scans_in_each_line[i]->points[j + 4].z +
                         scans_in_each_line[i]->points[j + 5].z;

      // 统计同一条线上每个点的特征
      // 这里用平均误差代表曲率大小，平均误差越大，越近似直线，曲率越小
      IdAndValue distance(j, sum_diffX * sum_diffX + sum_diffY * sum_diffY +
                                 sum_diffZ * sum_diffZ);
      cloud_curvature.push_back(distance);
    }

    // 将每条线分为6段
    for (int j = 0; j < 6; j++) {
      int sector_length = (int)(cloud_curvature.size() / 6);
      int sector_start = sector_length * j;
      int sector_end = sector_length * (j + 1) - 1;

      if (j == 5) {
        sector_end = cloud_curvature.size() - 1;
      }

      std::vector<IdAndValue> sub_cloud_curvature(
          cloud_curvature.begin() + sector_start,
          cloud_curvature.begin() + sector_end);

      extractFromSector(scans_in_each_line[i], sub_cloud_curvature, pc_out_edge,
                        pc_out_surf);
    }
  }
}

void FeatureExtraction::extractFromSector(
    const CloudPtr &pc_in, std::vector<IdAndValue> &cloud_curvature,
    CloudPtr &pc_out_edge, CloudPtr &pc_out_surf) {
  // 曲率小的排列在前
  std::sort(cloud_curvature.begin(), cloud_curvature.end(),
            [](const IdAndValue &a, const IdAndValue &b) {
              return a.value_ < b.value_;
            });

  int largest_picked_num = 0;
  int point_info_count = 0;

  /// 按照曲率最大的开始搜，选取曲率最大的角点
  std::vector<int> picked_points; // 标记被选中的角点，角点附近的点都不会被选取
  for (int i = cloud_curvature.size() - 1; i >= 0; i--) {

    int ind = cloud_curvature[i].id_;
    if (std::find(picked_points.begin(), picked_points.end(), ind) ==
        picked_points.end()) {
      if (cloud_curvature[i].value_ <= 0.1) {
        break;
      }

      largest_picked_num++; // 角点采集总数
      picked_points.push_back(ind);

      if (largest_picked_num <= 20) {
        pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      } else {
        break;
      }

      // 对角点同线附近的10个点进行角点检测
      for (int k = 1; k <= 5; k++) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
          break;
        }
        picked_points.push_back(ind + k);
      }

      for (int k = -1; k >= -5; k--) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
          break;
        }
        picked_points.push_back(ind + k);
      }
    }
  }

  // 剩下的点就做为平面点处理
  for (int i = 0; i < (int)cloud_curvature.size(); i++) {
    int index = cloud_curvature[i].id_;

    if (std::find(picked_points.begin(), picked_points.end(), index) ==
        picked_points.end()) {
      pc_out_surf->push_back(pc_in->points[index]);
    }
  }
}

} // namespace sad

#endif
