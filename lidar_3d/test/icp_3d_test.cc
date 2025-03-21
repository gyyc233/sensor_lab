#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include "icp_3d.h"
#include "sys_utils.h"

#include "sophus/se3.h"
#include "sophus/so3.h"

DEFINE_string(source, "./data/mapping_3d/milk.pcd", "第1个点云路径");
DEFINE_string(target, "./data/mapping_3d/milk.pcd", "第2个点云路径");
DEFINE_string(ground_truth_file, "./data/mapping_3d/kneeling_lady_pose.txt",
              "真值Pose");

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::ifstream fin(FLAGS_ground_truth_file);
  // 获取初始位姿
  Sophus::SE3 gt_pose;
  if (fin) {
    double tx, ty, tz, qw, qx, qy, qz;
    fin >> tx >> ty >> tz >> qw >> qx >> qx >> qy >> qz;
    fin.close();

    gt_pose = Sophus::SE3(Eigen::Quaterniond(qw, qx, qy, qz),
                          Eigen::Vector3d(tx, ty, tz));
  }

  // 将source 陪准到 target
  sad::CloudPtr source(new sad::PointCloudType),
      target(new sad::PointCloudType);
  pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
  pcl::io::loadPCDFile(fLS::FLAGS_target, *target);

  bool success;

  // P2P icp
  sad::evaluate_and_call(
      [&]() {
        sad::ICP_3D icp;
        icp.setSource(source);
        icp.setTarget(target);
        icp.setGroundTruth(gt_pose);

        SE3 pose;
        success = icp.AlignP2P(pose);
        if (success) {
          LOG(INFO) << "icp p2p align success, pose: (quaternion (x,y,z,w)) "
                    << pose.so3().unit_quaternion().coeffs().transpose()
                    << ", (translation): " << pose.translation().transpose();

          sad::CloudPtr source_trans(new sad::PointCloudType);
          // pcl::transformPointCloud 是右乘变换
          pcl::transformPointCloud(*source, *source_trans,
                                   pose.matrix().cast<float>());
          pcl::io::savePCDFileASCII("./data/mapping_3d/icp_trans.pcd",
                                    *source_trans);
        } else {
          LOG(ERROR) << "align error";
        }
      },
      "ICP P2P optimize", 1);

  // pcl icp
  sad::evaluate_and_call(
      [&]() {
        pcl::IterativeClosestPoint<sad::PointType, sad::PointType> icp_pcl;
        icp_pcl.setInputSource(source);
        icp_pcl.setInputTarget(target);
        sad::CloudPtr output_pcl(new sad::PointCloudType);
        icp_pcl.align(*output_pcl);
        Eigen::Matrix<float, 4, 4> rt = icp_pcl.getFinalTransformation();
        Eigen::Matrix3d r = rt.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d t = rt.block<3, 1>(0, 3).cast<double>();

        SE3 T(r, t);
        LOG(INFO) << "pose from icp pcl: "
                  << T.so3().unit_quaternion().coeffs().transpose() << ", "
                  << T.translation().transpose();
        pcl::io::savePCDFileASCII("./data/mapping_3d/icp_pcl_trans.pcd",
                                  *output_pcl);

        // 计算GT pose差异
        double pose_error = (gt_pose.inverse() * T).log().norm();
        LOG(INFO) << "ICP PCL pose error: " << pose_error;
      },
      "ICP PCL", 1);

  // P2L icp
  sad::evaluate_and_call(
      [&]() {
        sad::ICP_3D icp;
        icp.setSource(source);
        icp.setTarget(target);
        icp.setGroundTruth(gt_pose);

        SE3 pose;
        success = icp.AlignP2Line(pose);
        if (success) {
          LOG(INFO) << "icp p2l align success, pose: (quaternion (x,y,z,w)) "
                    << pose.so3().unit_quaternion().coeffs().transpose()
                    << ", (translation): " << pose.translation().transpose();

          sad::CloudPtr source_trans(new sad::PointCloudType);
          // pcl::transformPointCloud 是右乘变换
          pcl::transformPointCloud(*source, *source_trans,
                                   pose.matrix().cast<float>());
          pcl::io::savePCDFileASCII("./data/mapping_3d/icp_trans.pcd",
                                    *source_trans);
        } else {
          LOG(ERROR) << "align error";
        }
      },
      "ICP P2L optimize", 1);

  // P2Plane icp
  sad::evaluate_and_call(
      [&]() {
        sad::ICP_3D icp;
        icp.setSource(source);
        icp.setTarget(target);
        icp.setGroundTruth(gt_pose);

        SE3 pose;
        success = icp.AlignP2Plane(pose);
        if (success) {
          LOG(INFO)
              << "icp P2Plane align success, pose: (quaternion (x,y,z,w)) "
              << pose.so3().unit_quaternion().coeffs().transpose()
              << ", (translation): " << pose.translation().transpose();

          sad::CloudPtr source_trans(new sad::PointCloudType);
          // pcl::transformPointCloud 是右乘变换
          pcl::transformPointCloud(*source, *source_trans,
                                   pose.matrix().cast<float>());
          pcl::io::savePCDFileASCII("./data/mapping_3d/icp_trans.pcd",
                                    *source_trans);
        } else {
          LOG(ERROR) << "align error";
        }
      },
      "ICP P2Plane optimize", 1);

  return 0;
}
