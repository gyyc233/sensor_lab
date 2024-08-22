#ifndef __SENSORLAB_CTF_LK_OPTICAL_FLOW_H__
#define __SENSORLAB_CTF_LK_OPTICAL_FLOW_H__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <math.h>

namespace SensorLab {
class CtfLKOpticalFlow {
public:
  CtfLKOpticalFlow();

  ~CtfLKOpticalFlow();

  void inputParams(const char *pre_img_path, const char *cur_img_path);

  /// @brief 金字塔LK光流计算
  /// @param pre_img
  /// @param cur_img
  /// @param u x方向光流
  /// @param v y方向光流
  void getLucasKanadeOpticalFlow(cv::Mat &pre_img, cv::Mat &cur_img, cv::Mat &u,
                                 cv::Mat &v);

  void cv_single_of_api(const char *pre_img_path, const char *cur_img_path);

  /// @brief 从粗略到精细估计LK光流
  /// @param img1
  /// @param img2
  /// @param u x方向光流
  /// @param v y方向光流
  /// @param nLevels
  void coarseToFineEstimation(cv::Mat &img1, cv::Mat &img2, cv::Mat &u,
                              cv::Mat &v, int nLevels);

  /// @brief get max layer of image pyramid
  /// @param img image
  /// @return max layer of pyramid
  int getMaxLayer(cv::Mat &img);

  void draw_optical_flow(cv::Mat &img);

  std::vector<cv::Mat> getGaussianPyramid(cv::Mat &img, int nLevels);

private:
  cv::Mat getFx(cv::Mat &src1, cv::Mat &src2);

  cv::Mat getFy(cv::Mat &src1, cv::Mat &src2);

  cv::Mat getFt(cv::Mat &src1, cv::Mat &src2);

  bool isInsideImage(int y, int x, cv::Mat &m);

  double getSum9(cv::Mat &m, int y, int x);

  cv::Mat getSum9Mat(cv::Mat &m);

  void saveMat(cv::Mat &M, std::string s);

  cv::Mat u_;
  cv::Mat v_;
};
} // namespace SensorLab

#endif
