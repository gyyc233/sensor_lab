#include "matplotlibcpp.h"
#include "moving_lest_squares/moving_lest_squares.h"
#include <glog/logging.h>
#include <memory>

using namespace Algorithm;

namespace plt = matplotlibcpp;

int main() {

  std::vector<double> x_points = {0,   0.1, 0.2, 0.3, 0.4, 0.5,
                                  0.6, 0.7, 0.8, 0.9, 1.0};
  std::vector<double> y_points = {0, 4, 5, 14, 15, 14.5, 14, 12, 10, 5, 4};

  std::shared_ptr<MLSCurvedLine> mls_line_ptr =
      std::make_shared<MLSCurvedLine>();
  mls_line_ptr->setX(x_points);
  mls_line_ptr->setY(y_points);

  std::vector<double> test_x;
  std::vector<double> test_y;

  for (double i = 0; i <= 1.0; i += 0.01) {
    test_x.push_back(i);
    test_y.push_back(mls_line_ptr->fit(i));
  }

  plt::plot(test_x, test_y);
  plt::show();

  return 0;
}
