#include "cubic_spline/cubic_spline.h"
#include "spdlog/spdlog.h"
#include <memory>

using namespace Algorithm;

int main() {
  spdlog::set_level(spdlog::level::debug);

  std::vector<double> val_x = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> val_y = {sin(0), sin(1), sin(2), sin(3), sin(4),
                               sin(5), sin(6), sin(7), sin(8), sin(9)};

  std::shared_ptr<CubicSplineOperator> cubic_spline_operator =
      std::make_shared<CubicSplineOperator>();

  cubic_spline_operator->setSamplePoints(val_x, val_y);
  cubic_spline_operator->cubicSplineNatural();

  std::vector<double> input_val = {3, 5.5};
  std::vector<double> predicated_val;
  cubic_spline_operator->cubicSplineFit(input_val, predicated_val);
  return 0;
}