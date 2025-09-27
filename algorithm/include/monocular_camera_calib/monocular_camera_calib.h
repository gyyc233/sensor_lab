#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

namespace Algorithm {
class MonocularCameraCalib {
public:
  /// @brief 单目相机标定
  /// @param intrinsic_filename 输出的相机内参标定结果
  /// @param pics_path 标定图片路径
  void SingleCalibrate(std::string intrinsic_filename = "intrinsics.yml",
                       std::string pics_path = "single_calib_pic");
  //将要读取的图片路径存储在fileList中
  void InitFileList(std::string path, std::vector<std::string> &files);
};
} // namespace Algorithm
