#ifndef __SENSORLAB_HS_LK_OPTICAL_FLOW_H__
#define __SENSORLAB_HS_LK_OPTICAL_FLOW_H__

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <math.h>

namespace SensorLab {
class HSOpticalFlow {
public:
  HSOpticalFlow();

  ~HSOpticalFlow();

  void inputParams(const char *pre_img_path, const char *cur_img_path);

  void getHornSchunckOpticalFlow(cv::Mat img1, cv::Mat img2);

  void draw_optical_flow(cv::Mat &img);

private:
  cv::Mat getFx(cv::Mat &src1, cv::Mat &src2);

  cv::Mat getFy(cv::Mat &src1, cv::Mat &src2);

  cv::Mat getFt(cv::Mat &src1, cv::Mat &src2);

  bool isInsideImage(int y, int x, cv::Mat &m);

  double getAverage4(cv::Mat &m, int y, int x);

  cv::Mat getAverage4Mat(cv::Mat &m);

  void saveMat(cv::Mat &M, std::string s);

  cv::Mat u_;
  cv::Mat v_;
};
} // namespace SensorLab

#endif
