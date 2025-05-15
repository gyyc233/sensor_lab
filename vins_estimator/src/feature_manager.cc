#ifndef ROS_CATKIN
#define ROS_CATKIN

#include "feature_manager.h"

// 该模块在整个VIO系统中负责特征点的生命周期管理、深度估计、滑动窗口更新、视差检测等核心功能
// 是连接前端视觉特征跟踪和后端非线性优化之间的桥梁

// 1. 特征点生命周期管理
// 添加新特征点并记录其在每一帧中的观测
// 判断哪些特征点可用于三角化或优化
// 移除无效特征点（如被边缘化的、追踪失败的、异常点等）

// 2. 深度估计与三角化
// 使用 SVD 方法对特征点进行三角化，计算其深度值
// 将深度信息参与优化，并根据优化结果更新深度

// 3. 滑动窗口机制支持，支持三种滑动方式
// removeBack()：移除最老帧的信息
// removeFront(int frame_count)：移除指定帧前的观测
// removeBackShiftDepth(...)：移除旧帧的同时调整深度值以保持一致性

// 4. 视差检测
//  compensatedParallax2(...)（补偿视差计算），用于判断特征点是否具有足够的视差用于初始化

namespace sensor_lab {
int FeaturePerId::endFrame() {
  return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Eigen::Matrix3d _Rs[]) : Rs(_Rs) {
  for (int i = 0; i < NUM_OF_CAM; i++)
    ric[i].setIdentity();
}

void FeatureManager::setRic(Eigen::Matrix3d _ric[]) {
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ric[i] = _ric[i];
  }
}

void FeatureManager::clearState() { feature.clear(); }

int FeatureManager::getFeatureCount() {
  int cnt = 0;
  for (auto &it : feature) {

    it.used_num = it.feature_per_frame.size();

    if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2) {
      cnt++;
    }
  }
  return cnt;
}

// VIO的图像处理主循环中，每次接收到新的图像特征时会调用此函数
bool FeatureManager::addFeatureCheckParallax(
    int frame_count,
    const std::map<
        int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
    double td) {
  // ROS_DEBUG("input feature: %d", (int)image.size());
  // ROS_DEBUG("valid num of feature: %d", getFeatureCount());
  double parallax_sum = 0; // 视差总和
  int parallax_num = 0;    // 参与视差计算的特征点个数
  int last_track_num = 0;  // 上一帧追踪到的特征点数量

  // 遍历输入的特征点
  for (auto &id_pts : image) {
    // 1. 对每一个输入的特征点 ID，创建一个新的 FeaturePerFrame
    FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

    int feature_id = id_pts.first;
    // 2. 若该特征点不存在，则创建一个新的 FeaturePerId，否则追加当前帧观测信息
    auto it = find_if(feature.begin(), feature.end(),
                      [feature_id](const FeaturePerId &it) {
                        return it.feature_id == feature_id;
                      });

    if (it == feature.end()) {
      feature.push_back(FeaturePerId(feature_id, frame_count));
      feature.back().feature_per_frame.push_back(f_per_fra);
    } else if (it->feature_id == feature_id) {
      it->feature_per_frame.push_back(f_per_fra);
      last_track_num++;
    }
  }

  // 历史帧数量<2或当前帧追踪的特征点数量小于20 不作视差检测
  if (frame_count < 2 || last_track_num < 20)
    return true;

  // estimate parallax 估计视差
  for (auto &it_per_id : feature) {
    // 某个特征点至少被连续两帧观测到
    if (it_per_id.start_frame <= frame_count - 2 &&
        it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >=
            frame_count - 1) {
      // 累加视差值并统计参与计算的特征点个数
      parallax_sum += compensatedParallax2(it_per_id, frame_count);
      parallax_num++;
    }
  }

  if (parallax_num == 0) {
    return true;
  } else {
    // 计算平均视差，乘以焦距转为像素
    std::cout << "parallax_sum: " << parallax_sum
              << " parallax_num: " << parallax_num << std::endl;
    std::cout << "current parallax: "
              << parallax_sum / parallax_num * FOCAL_LENGTH << std::endl;

    // 当前帧是否具有足够运动来支持初始化
    return parallax_sum / parallax_num >= MIN_PARALLAX;
  }
}

void FeatureManager::debugShow() {
  ROS_DEBUG("debug show");
  for (auto &it : feature) {
    ROS_ASSERT(it.feature_per_frame.size() != 0);
    ROS_ASSERT(it.start_frame >= 0);
    ROS_ASSERT(it.used_num >= 0);

    // 特征点唯一id，当前被使用次数，特征点首次出现的帧号
    ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
    int sum = 0;
    for (auto &j : it.feature_per_frame) {
      ROS_DEBUG("%d,", int(j.is_used));
      sum += j.is_used;
      printf("(%lf,%lf) ", j.point(0), j.point(1));
    }
    ROS_ASSERT(it.used_num == sum);
  }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
  for (auto &it : feature) {
    if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
      Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
      int idx_l = frame_count_l - it.start_frame;
      int idx_r = frame_count_r - it.start_frame;

      a = it.feature_per_frame[idx_l].point;

      b = it.feature_per_frame[idx_r].point;

      corres.push_back(std::make_pair(a, b));
    }
  }
  return corres;
}

void FeatureManager::setDepth(const Eigen::VectorXd &x) {
  int feature_index = -1;
  for (auto &it_per_id : feature) {
    // 统计该特征点被多少帧观测到
    it_per_id.used_num = it_per_id.feature_per_frame.size();

    // 至少被滑动窗口中靠前的2帧观测到，再进行深度更新
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    // 逆深度取反
    it_per_id.estimated_depth = 1.0 / x(++feature_index);
    if (it_per_id.estimated_depth < 0) {
      it_per_id.solve_flag = 2; // 无效深度
    } else
      it_per_id.solve_flag = 1;
  }
}

void FeatureManager::removeFailures() {
  for (auto it = feature.begin(); it != feature.end();) {
    if (it->solve_flag == 2)
      it = feature.erase(it);
    else
      ++it;
  }
}

void FeatureManager::clearDepth(const Eigen::VectorXd &x) {
  int feature_index = -1;
  for (auto &it_per_id : feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    it_per_id.estimated_depth = 1.0 / x(++feature_index);
  }
  // 这里没有 solve_flag
}

// 该函数通常在
// 非线性优化前调用，将当前所有有效特征点的深度信息打包成向量传给优化器
// 优化器会根据这些初始深度值进行联合优化（包括相机位姿和特征点深度）
// 逆深度有助于提升数值稳定性，特别是在深度未知或变化剧烈的情况下
VectorXd FeatureManager::getDepthVector() {
  Eigen::VectorXd dep_vec(getFeatureCount());
  int feature_index = -1;
  for (auto &it_per_id : feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
#if 1
    // 使用逆深度
    dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
    // 使用真实深度
    dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
  }
  return dep_vec;
}

void FeatureManager::triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[],
                                 Matrix3d ric[]) {
  for (auto &it_per_id : feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    // 若该特征点已有正深度则跳过三角化
    if (it_per_id.estimated_depth > 0)
      continue;

    int imu_i =
        it_per_id.start_frame; // 当前特征点首次被观测到的帧索引，作为参考帧
    int imu_j = imu_i - 1; // 方便下面imu_j++，直接从imu_i帧开始

    ROS_ASSERT(NUM_OF_CAM == 1);
    // it_per_id.feature_per_frame.size()：表示该特征点在滑动窗口中被多少帧观测到，每帧包含(u,v)两个归一化坐标约束
    // 乘2 4 表示特征点在世界坐标系下的齐次坐标
    Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
    int svd_idx = 0;

    Eigen::Matrix<double, 3, 4> P0;
    // 将imu坐标系下的相机偏移tic[0]转换到世界坐标系下
    // tic[0] 相机相对于 IMU 的平移外参
    // Rs[imu_i] 是 IMU 在世界坐标系下的旋转
    // Ps[imu_i] 是 IMU 在世界坐标系下的平移
    // 得到的是相机在世界坐标系下的平移向量 t0
    Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];

    // 计算当前帧 IMU 到相机的旋转变换
    // ric[0] 是相机相对于 IMU 的旋转外参
    // Rs[imu_i] 是 IMU 在世界坐标系下的旋转
    // 相乘后得到的是相机在世界坐标系下的旋转矩阵 R0
    Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];

    // 参考帧
    P0.leftCols<3>() = Eigen::Matrix3d::Identity();
    P0.rightCols<1>() = Eigen::Vector3d::Zero();

    // 遍历当前特征点 it_per_id 在不同图像帧中的所有观测数据
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++; // 当前帧在滑动窗口中的imu帧索引

      // 当前帧相机在世界坐标系下的RT
      Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
      Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

      // 计算参考帧到当前帧的RT
      Eigen::Vector3d t = R0.transpose() * (t1 - t0);
      Eigen::Matrix3d R = R0.transpose() * R1;

      // 当前帧到参考帧的变换
      // R.transpose() 当前帧到参考帧的旋转
      // -R.transpose() * t 当前帧到参考着的平移
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = R.transpose();
      P.rightCols<1>() = -R.transpose() * t;

      // 获取当前帧中该特征点的归一化图像坐标并进行归一化处理
      Eigen::Vector3d f = it_per_frame.point.normalized();
      // 通过向量平行，叉乘为0 构造2个约束
      svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
      svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

      if (imu_i == imu_j)
        continue;
    }

    ROS_ASSERT(svd_idx == svd_A.rows());
    // svd分解 取右奇异矩阵V的最后一列
    Eigen::Vector4d svd_V =
        Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV)
            .matrixV()
            .rightCols<1>();
    double svd_method = svd_V[2] / svd_V[3]; // 该点深度估计

    // 三角化阶段：通常使用真实深度初始化
    // 非线性优化阶段：通常使用逆深度表示，提升数值稳定性（尤其是远距离点）
    // 边缘化或滑动窗口更新时：也常使用逆深度进行变换保持一致性；

    // it_per_id->estimated_depth = -b / A;
    // it_per_id->estimated_depth = svd_V[2] / svd_V[3];

    it_per_id.estimated_depth = svd_method;
    // it_per_id->estimated_depth = INIT_DEPTH;

    if (it_per_id.estimated_depth < 0.1) {
      it_per_id.estimated_depth = INIT_DEPTH;
    }
  }
}

void FeatureManager::removeOutlier() {
  for (auto it = feature.begin(); it != feature.end();) {
    if (it->is_outlier && it->used_num > 0)
      it = feature.erase(it);
    else
      ++it;
  }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R,
                                          Eigen::Vector3d marg_P,
                                          Eigen::Matrix3d new_R,
                                          Eigen::Vector3d new_P) {
  // 遍历所有特征点
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    // 该特征点首次出现的帧号不是最老帧
    if (it->start_frame != 0)
      it->start_frame--; //
    else {
      // 该特征点首次出现的帧号在最老帧
      Eigen::Vector3d uv_i =
          it->feature_per_frame[0].point; // 保存最早帧的归一化图像坐标 uv_i
      it->feature_per_frame.erase(
          it->feature_per_frame.begin()); // 删除最早帧对该特征点的观测

      // 如果删除后只剩一个观测帧，无法再进行三角化或优化，直接删除该特征点
      if (it->feature_per_frame.size() < 2) {
        feature.erase(it);
        continue;
      } else { // 深度一致性维护
        Eigen::Vector3d pts_i =
            uv_i * it->estimated_depth; // 重建该点在旧参考帧下的3D坐标
        Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P; // 转到世界坐标系
        Eigen::Vector3d pts_j =
            new_R.transpose() * (w_pts_i - new_P); // 转到新参考帧下

        // 更新深度
        double dep_j = pts_j(2);
        if (dep_j > 0)
          it->estimated_depth = dep_j;
        else
          it->estimated_depth = INIT_DEPTH;
      }
    }
  }
}

void FeatureManager::removeBack() {
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    // 特征点首次被观测的帧号大于 0，说明它不是由即将被移除的帧首次观测到的
    if (it->start_frame != 0)
      it->start_frame--;
    else {
      // 是最老帧首次观测到的特征点

      // 删除该特征点在最早帧中的观测信息
      it->feature_per_frame.erase(it->feature_per_frame.begin());

      // 如果删除后该特征点没有任何观测帧了，就将其从 feature 列表中彻底删除
      // 否则保留下来，等待后续优化或边缘化处理
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

void FeatureManager::removeFront(int frame_count) {
  // 希望保留最新的几帧进行优化，而丢弃更早的关键帧

  // 删除所有在 frame_count 帧及之前对该特征点的观测，同时更新其起始帧；
  // 如果特征点已无观测，则从系统中清除
  for (auto it = feature.begin(), it_next = feature.begin();
       it != feature.end(); it = it_next) {
    it_next++;

    if (it->start_frame == frame_count) {
      it->start_frame--;
    } else {
      int j = WINDOW_SIZE - 1 - it->start_frame;
      if (it->endFrame() < frame_count - 1)
        continue;

      // 删除指定帧前的观测帧
      it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
      if (it->feature_per_frame.size() == 0)
        feature.erase(it);
    }
  }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id,
                                            int frame_count) {
  // check the second last frame is keyframe or not
  // parallax betwwen seconde last frame and third last frame
  // 计算某特征点在倒数第二帧和倒数第三帧之间的视差
  const FeaturePerFrame &frame_i =
      it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
  const FeaturePerFrame &frame_j =
      it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

  double ans = 0;
  Eigen::Vector3d p_j = frame_j.point;

  double u_j = p_j(0);
  double v_j = p_j(1);

  Eigen::Vector3d p_i = frame_i.point;
  Eigen::Vector3d p_i_comp;

  // 可选优化方案：将倒数第三帧的观测投影到倒数第二帧下进行比较，再计算视视差
  // int r_i = frame_count - 2;
  // int r_j = frame_count - 1;
  // p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] *
  // ric[camera_id_i] * p_i;
  p_i_comp = p_i; // du_comp == du
  double dep_i = p_i(2);
  double u_i = p_i(0) / dep_i;
  double v_i = p_i(1) / dep_i;
  // 计算视差
  double du = u_i - u_j, dv = v_i - v_j;

  double dep_i_comp = p_i_comp(2);
  double u_i_comp = p_i_comp(0) / dep_i_comp;
  double v_i_comp = p_i_comp(1) / dep_i_comp;
  double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

  ans = max(
      ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}

} // namespace sensor_lab

#endif
