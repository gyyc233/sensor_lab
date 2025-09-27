#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {
using namespace std;
using namespace cv;

class StereoCalib {
public:
  void initFileList(string dir, int first, int last);

  int stereoCalibrate(string intrinsic_filename, string extrinsic_filename);

  /// @brief 双目立体匹配和测量
  /// @param picNum
  /// @param intrinsic_filename
  /// @param extrinsic_filename
  /// @param no_display
  /// @param point_cloud_filename
  /// @return
  int stereoMatch(int picNum, string intrinsic_filename,
                  string extrinsic_filename, bool no_display,
                  string point_cloud_filename);

private:
  /// @brief 生成点云后保存
  /// @param filename
  /// @param mat
  void saveXYZ(string filename, const Mat &mat);

  /// @brief 保存视差数据
  /// @param filename
  /// @param mat
  void saveDisp(const string filename, const Mat &mat);

  void F_Gray2Color(Mat gray_mat, Mat &color_mat);

  Mat F_mergeImg(Mat img1, Mat disp8);

  void cvMatToPcl(cv::Mat &mat);

  cv::Size img_size;
  cv::Size pat_size; //每张棋盘寻找的角点个数是7*6个
  const double patLen = 30.0f; // unit: mm  标定板每个格的宽度（金属标定板）
  double imgScale = 1.0; //图像缩放的比例因子
  //将要读取的图片路径存储在fileList中
  vector<string> fileList;
};
} // namespace Algorithm
