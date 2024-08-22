#ifndef __SENSORLAB_LK_OPTICAL_FLOW_H__
#define __SENSORLAB_LK_OPTICAL_FLOW_H__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <math.h>

namespace SensorLab {
class LKOpticalFlow {
public:
  LKOpticalFlow();

  ~LKOpticalFlow();

  void inputParams(const char *pre_img_path, const char *cur_img_path);

  /// @brief LK光流计算
  /// @param pre_img
  /// @param cur_img
  /// @param u x方向光流
  /// @param v y方向光流
  void getLucasKanadeLKOpticalFlow(cv::Mat &pre_img, cv::Mat &cur_img,
                                   cv::Mat &u, cv::Mat &v);

  void cv_single_of_api(const char *pre_img_path, const char *cur_img_path);

  void draw_optical_flow(cv::Mat &img);

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
