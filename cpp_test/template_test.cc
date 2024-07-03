#include <feature.hpp>
#include <glog/logging.h>
#include <vector>

int main() {
  std::vector<double> vec = {1, 2, 3, 5, 7, 11, 13};
  std::cout << "future: "
            << getFuture<double, std::vector<double>::iterator>(vec.begin(),
                                                                vec.end())
            << std::endl;

  std::cout << "biased var: "
            << getBiasedVariance<double, std::vector<double>::iterator>(
                   vec.begin(), vec.end())
            << std::endl;

  std::cout
      << "biased standard dev: "
      << getBiasedStandardDeviation<double, std::vector<double>::iterator>(
             vec.begin(), vec.end())
      << std::endl;

  std::cout << "unbiased var: "
            << getUnbiasedVariance<double, std::vector<double>::iterator>(
                   vec.begin(), vec.end())
            << std::endl;

  std::cout
      << "unbiased standard dev: "
      << getUnbiasedStandardDeviation<double, std::vector<double>::iterator>(
             vec.begin(), vec.end())
      << std::endl;
  return 0;
}
