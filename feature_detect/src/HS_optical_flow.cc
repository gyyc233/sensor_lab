#include "HS_optical_flow.h"

using namespace SensorLab;

HSOpticalFlow::HSOpticalFlow() {}

HSOpticalFlow::~HSOpticalFlow() {}

cv::Mat HSOpticalFlow::getFx(cv::Mat &src1, cv::Mat &src2) {
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

cv::Mat HSOpticalFlow::getFy(cv::Mat &src1, cv::Mat &src2) {
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

cv::Mat HSOpticalFlow::getFt(cv::Mat &src1, cv::Mat &src2) {
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

bool HSOpticalFlow::isInsideImage(int y, int x, cv::Mat &m) {
  if (x >= 0 && x < m.cols && y >= 0 && y < m.rows)
    return true;
  else
    return false;
}

double HSOpticalFlow::getAverage4(cv::Mat &m, int y, int x) {
  if (x < 0 || x >= m.cols)
    return 0;
  if (y < 0 || y >= m.rows)
    return 0;

  double val = 0.0;
  int tmp = 0;
  if (isInsideImage(y - 1, x, m)) {
    ++tmp;
    val += m.at<double>(y - 1, x);
  }
  if (isInsideImage(y + 1, x, m)) {
    ++tmp;
    val += m.at<double>(y + 1, x);
  }
  if (isInsideImage(y, x - 1, m)) {
    ++tmp;
    val += m.at<double>(y, x - 1);
  }
  if (isInsideImage(y, x + 1, m)) {
    ++tmp;
    val += m.at<double>(y, x + 1);
  }
  return val / tmp;
}

cv::Mat HSOpticalFlow::getAverage4Mat(cv::Mat &m) {
  cv::Mat res = cv::Mat::zeros(m.rows, m.cols, CV_64FC1);
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      res.at<double>(i, j) = getAverage4(m, i, j);
    }
  }
  return res;
}

void HSOpticalFlow::saveMat(cv::Mat &M, std::string s) {
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

void HSOpticalFlow::getHornSchunckOpticalFlow(cv::Mat img1, cv::Mat img2) {
  double lambda = 0.05;
  cv::Mat u = cv::Mat::zeros(img1.rows, img1.cols, CV_64FC1);
  cv::Mat v = cv::Mat::zeros(img1.rows, img1.cols, CV_64FC1);

  cv::Mat fx = getFx(img1, img2);
  cv::Mat fy = getFy(img1, img2);
  cv::Mat ft = getFt(img1, img2);

  int i = 0;
  double last = 0.0;
  while (1) {
    cv::Mat Uav = getAverage4Mat(u);
    cv::Mat Vav = getAverage4Mat(v);
    cv::Mat P = fx.mul(Uav) + fy.mul(Vav) + ft;
    cv::Mat D = fx.mul(fx) + fy.mul(fy) + lambda;
    cv::Mat tmp;
    divide(P, D, tmp);
    cv::Mat utmp, vtmp;
    utmp = Uav - fx.mul(tmp);
    vtmp = Vav - fy.mul(tmp);
    cv::Mat eq = fx.mul(utmp) + fy.mul(vtmp) + ft;
    double this_time = mean(eq)[0];
    std::cout << "i = " << i << ", mean = " << this_time << std::endl;
    if (i != 0 && fabs(last) <= fabs(this_time))
      break;
    i++;
    last = this_time;
    u = utmp;
    v = vtmp;
  }
  saveMat(u, "U");
  saveMat(v, "V");
  u_ = u;
  v_ = v;
}

void HSOpticalFlow::draw_optical_flow(cv::Mat &img) {
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

  cv::imshow("Frame", img);
  cv::waitKey(0);
  cv::destroyAllWindows();
}

void HSOpticalFlow::inputParams(const char *pre_img_path,
                                const char *cur_img_path) {
  cv::Mat pre_img = cv::imread(pre_img_path, cv::IMREAD_GRAYSCALE);
  cv::Mat cur_img = cv::imread(cur_img_path, cv::IMREAD_GRAYSCALE);

  cv::Mat pre_frame = pre_img(cv::Rect(0, 0, 640, 480));
  cv::Mat cur_frame = cur_img(cv::Rect(0, 0, 640, 480));

  std::cout << pre_frame.rows << ", " << pre_frame.cols << std::endl;

  pre_frame.convertTo(pre_frame, CV_64FC1, 1.0 / 255, 0);
  cur_frame.convertTo(cur_frame, CV_64FC1, 1.0 / 255, 0);

  getHornSchunckOpticalFlow(pre_frame, cur_frame);
}
