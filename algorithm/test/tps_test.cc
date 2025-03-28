#include "tps/tps.h"
using namespace Algorithm;

int main() {
  std::shared_ptr<TPSOperator> tps_operator = std::make_shared<TPSOperator>();

  std::vector<cv::Point2f> points_source = {
      cv::Point2f(1, 1), cv::Point2f(2, 2), cv::Point2f(3, 3),
      cv::Point2f(4, 4)};
  std::vector<cv::Point2f> points_target = {
      cv::Point2f(1, 2), cv::Point2f(2, 3), cv::Point2f(3, 4),
      cv::Point2f(4, 5)};

  cv::Mat K;
  tps_operator->calculate_K(points_source, K);

  cv::Mat L;
  tps_operator->calculate_L(points_source, L);

  cv::Mat W;
  tps_operator->calculate_W(points_source, points_target, W);

  cv::Mat target_x, target_y;
  tps_operator->tps_map(points_source, W, 6, 6, target_x, target_y);

  std::cout << "target_x:\n" << target_x << std::endl;
  std::cout << "target_y:\n" << target_y << std::endl;

  return 0;
}
