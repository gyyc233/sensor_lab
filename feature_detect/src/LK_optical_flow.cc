#include "LK_optical_flow.h"
#include <opencv2/video/tracking.hpp>

using namespace SensorLab;

LKOpticalFlow::LKOpticalFlow() {}

LKOpticalFlow::~LKOpticalFlow() {}

void LKOpticalFlow::inputParams(const char *pre_img_path,
                                const char *cur_img_path) {
  cv::Mat pre_img = cv::imread(pre_img_path, cv::IMREAD_GRAYSCALE);
  cv::Mat cur_img = cv::imread(cur_img_path, cv::IMREAD_GRAYSCALE);

  assert(pre_img.data != nullptr && cur_img.data != nullptr);

  pre_img.convertTo(pre_img, CV_64FC1, 1.0 / 255, 0);
  cur_img.convertTo(cur_img, CV_64FC1, 1.0 / 255, 0);

  cv::Mat u = cv::Mat::zeros(pre_img.rows, pre_img.cols, CV_64FC1);
  cv::Mat v = cv::Mat::zeros(cur_img.rows, cur_img.cols, CV_64FC1);
  std::cout << "pre_img: rows " << pre_img.rows << ", cols " << pre_img.cols
            << std::endl;

  getLucasKanadeLKOpticalFlow(pre_img, cur_img, u, v);
}

void LKOpticalFlow::getLucasKanadeLKOpticalFlow(cv::Mat &pre_img,
                                                cv::Mat &cur_img, cv::Mat &u,
                                                cv::Mat &v) {
  cv::Mat fx = get_fx(pre_img, cur_img);
  cv::Mat fy = get_fy(pre_img, cur_img);
  cv::Mat ft = get_ft(pre_img, cur_img);

  cv::Mat fx2 = fx.mul(fx);
  cv::Mat fy2 = fy.mul(fy);
  cv::Mat fx_fy = fx.mul(fy);
  cv::Mat fx_ft = fx.mul(ft);
  cv::Mat fy_ft = fy.mul(ft);

  cv::Mat sum_fx2 = get_sum9_mat(fx2);
  cv::Mat sum_fy2 = get_sum9_mat(fy2);
  cv::Mat sum_fx_ft = get_sum9_mat(fx_ft);
  cv::Mat sum_fx_fy = get_sum9_mat(fx_fy);
  cv::Mat sum_fy_ft = get_sum9_mat(fy_ft);

  // TODO: 这里 u v 的计算方法不太明白，还是直接使用CV的光流接口吧
  cv::Mat det =
      sum_fx2.mul(sum_fy2) - sum_fx_fy.mul(sum_fx_fy); // A的行列式计算（二阶）
  u = sum_fx_fy.mul(sum_fy_ft) - sum_fy2.mul(sum_fx_ft); //算出u*det
  v = sum_fx_ft.mul(sum_fx_fy) - sum_fx2.mul(sum_fy_ft); //算出v*det

  cv::divide(u, det, u); // u=u/det
  cv::divide(v, det, v); // v=v/det

  saveMat(u, "U");
  saveMat(v, "V");

  u_ = u;
  v_ = v;
}

cv::Mat LKOpticalFlow::get_fx(cv::Mat &src1, cv::Mat &src2) {
  // 两张图的梯度再相加，是不是不相加也可以？
  cv::Mat fx;
  cv::Mat kernel = cv::Mat::ones(2, 2, CV_64FC1);
  kernel.at<double>(0, 0) = -1.0;
  kernel.at<double>(1, 0) = -1.0;

  cv::Mat dst1, dst2;
  cv::filter2D(src1, dst1, -1, kernel);
  cv::filter2D(src2, dst2, -1, kernel);

  fx = dst1 + dst2;
  return fx;
}

cv::Mat LKOpticalFlow::get_fy(cv::Mat &src1, cv::Mat &src2) {
  cv::Mat fy;
  cv::Mat kernel = cv::Mat::ones(2, 2, CV_64FC1);
  kernel.at<double>(0, 0) = -1.0;
  kernel.at<double>(0, 1) = -1.0;

  cv::Mat dst1, dst2;
  cv::filter2D(src1, dst1, -1, kernel);
  cv::filter2D(src2, dst2, -1, kernel);

  fy = dst1 + dst2;
  return fy;
}

cv::Mat LKOpticalFlow::get_ft(cv::Mat &src1, cv::Mat &src2) {
  cv::Mat ft;
  cv::Mat kernel = cv::Mat::ones(2, 2, CV_64FC1);
  kernel = kernel.mul(-1);

  cv::Mat dst1, dst2;
  cv::filter2D(src1, dst1, -1, kernel);
  kernel = kernel.mul(-1);
  cv::filter2D(src2, dst2, -1, kernel);

  ft = dst1 + dst2;
  return ft;
}

bool LKOpticalFlow::is_inside_image(int y, int x, cv::Mat &m) {
  if (x >= 0 && x < m.cols && y >= 0 && y < m.rows)
    return true;
  else
    return false;
}

double LKOpticalFlow::get_sum_9(cv::Mat &m, int y, int x) {
  if (x < 0 || x >= m.cols)
    return 0;
  if (y < 0 || y >= m.rows)
    return 0;

  double val = 0.0;
  int tmp = 0;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      if (is_inside_image(y + i, x + j, m)) {
        ++tmp;
        val += m.at<double>(y + i, x + j);
      }
    }
  }
  if (tmp == 9)
    return val;
  else
    return m.at<double>(y, x) * 9;
}

cv::Mat LKOpticalFlow::get_sum9_mat(cv::Mat &m) {
  cv::Mat res = cv::Mat::zeros(m.rows, m.cols, CV_64FC1);
  for (int i = 1; i < m.rows - 1; i++) {
    for (int j = 1; j < m.cols - 1; j++) {
      res.at<double>(i, j) = get_sum_9(m, i, j);
    }
  }

  return res;
}

void LKOpticalFlow::saveMat(cv::Mat &M, std::string s) {
  s += ".txt";
  FILE *pOut = fopen(s.c_str(), "w+");
  for (int i = 0; i < M.rows; i++) {
    for (int j = 0; j < M.cols; j++) {
      fprintf(pOut, "%lf", M.at<double>(i, j));
      if (j == M.cols - 1)
        fprintf(pOut, "\n");
      else
        fprintf(pOut, " ");
    }
  }
  fclose(pOut);
}

void LKOpticalFlow::cv_single_of_api(const char *pre_img_path,
                                     const char *cur_img_path) {
  cv::Mat pre_img = cv::imread(pre_img_path, -1);
  cv::Mat cur_img = cv::imread(cur_img_path, -1);

  cv::Mat pre_frame, cur_frame;
  cv::cvtColor(pre_img, pre_frame, cv::COLOR_BGR2GRAY);
  cv::cvtColor(cur_img, cur_frame, cv::COLOR_BGR2GRAY);

  assert(pre_frame.data != nullptr && cur_img.data != nullptr);

  std::vector<cv::Point2f> p0, p1;
  cv::goodFeaturesToTrack(pre_frame, p0, 100, 0.3, 7, cv::Mat(), 7, false,
                          0.04);

  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(pre_frame, cur_frame, p0, p1, status, err,
                           cv::Size(15, 15), 2);
  // Create a mask image for drawing purposes

  // Create some random colors
  std::vector<cv::Scalar> colors;
  cv::RNG rng;
  for (int i = 0; i < 100; i++) {
    int r = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int b = rng.uniform(0, 256);
    colors.push_back(cv::Scalar(r, g, b));
  }

  cv::Mat mask = cv::Mat::zeros(pre_img.size(), pre_img.type());
  std::vector<cv::Point2f> good_new;
  for (uint i = 0; i < p0.size(); i++) {
    // Select good points
    if (status[i] == 1) {
      good_new.push_back(p1[i]);
      // draw the tracks
      cv::line(mask, p1[i], p0[i], colors[i], 2);
      cv::circle(cur_img, p1[i], 5, colors[i], -1);
    }
  }

  cv::Mat img;
  cv::add(pre_img, mask, img);
  cv::imshow("Frame", img);
  cv::waitKey(0);
  cv::destroyAllWindows();
}

void LKOpticalFlow::draw_optical_flow(cv::Mat &img) {
  int width = img.cols;  // y
  int height = img.rows; // x

  int new_i, new_j;                //加入光流后的新坐标
  for (int i = 0; i < height; i++) //遍历x
  {
    for (int j = 0; j < width; j++) //遍历y
    {
      new_i = i + int(u_.at<double>(i, j)); //加入光流之后的新x坐标
      new_j = j + int(v_.at<double>(i, j)); //加入光流之后的新y坐标

      if (new_i >= 0 && new_i < height && new_j >= 0 && new_j < width) {
        if ((u_.at<double>(i, j) + v_.at<double>(i, j) > 20) &&
            (u_.at<double>(i, j) + v_.at<double>(i, j) < 40)) {
          circle(img, cv::Point(new_j, new_i), 2,
                 cv::Scalar(0, 255, 0)); //在新坐标点画圆
          line(img, cv::Point(j, i), cv::Point(new_j, new_i),
               cv::Scalar(0, 255, 0)); //两个点之间画线
        }
      }
    }
  }
}
