#include "moving_lest_squares/moving_lest_squares.h"
#include <glog/logging.h>
#include <memory>

using namespace Algorithm;

int main() {

  std::vector<double> x_points = {0,   0.1, 0.2, 0.3, 0.4, 0.5,
                                  0.6, 0.7, 0.8, 0.9, 1.0};
  std::vector<double> y_points = {0, 4, 5, 14, 15, 14.5, 14, 12, 10, 5, 4};

  std::shared_ptr<MLSCurvedLine> mls_line_ptr =
      std::make_shared<MLSCurvedLine>();
  mls_line_ptr->SetX(x_points);
  mls_line_ptr->SetY(y_points);

  std::string nums;
  std::cin >> nums;

  std::cout << mls_line_ptr->fit(std::stod(nums)) << std::endl;

  return 0;
}
