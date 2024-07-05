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

  std::vector<std::vector<double>> data{
      {1.2, 2.5, 5.6, -2.5}, {-3.6, 9.2, 0.5, 7.2}, {4.3, 1.3f, 9.4, -3.4}};

  std::vector<std::vector<double>> covariance;
  std::vector<double> mean;
  calculateCovariance<double>(data, covariance, mean);

  return 0;
}
