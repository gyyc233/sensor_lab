#include "least_squares/least_squares.h"
#include <glog/logging.h>
#include <memory>

using namespace Algorithm;

int main() {
  std::shared_ptr<LeastSquare> least_square_operator =
      std::make_shared<LeastSquare>(3);

  least_square_operator->inputX({0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50});
  least_square_operator->inputY(
      {0, 1.27, 2.16, 2.86, 3.44, 3.87, 4.15, 4.37, 4.51, 4.58, 4.02});
  least_square_operator->solveViaAlgebraic();
  least_square_operator->solveViaMatrix();
  return 0;
}
