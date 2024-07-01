#include <feature.hpp>
#include <vector>

int main() {
  std::vector<double> vec = {1, 2, 3, 5, 7, 11, 13};
  std::cout << getFuture<double, std::vector<double>::iterator>(vec.begin(),
                                                                vec.end())
            << std::endl;

  std::cout << getVariance<double, std::vector<double>::iterator>(vec.begin(),
                                                                  vec.end())
            << std::endl;

  std::cout << getStandardDeviation<double, std::vector<double>::iterator>(
                   vec.begin(), vec.end())
            << std::endl;
  return 0;
}
