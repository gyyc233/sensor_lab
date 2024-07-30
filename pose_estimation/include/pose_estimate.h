#include "my_time.h"
#include <iostream>

namespace SensorLab {
class PoseEstimation {
public:
  PoseEstimation() { std::cout << "construct PoseEstimation" << std::endl; }
  virtual ~PoseEstimation() {
    std::cout << "destruct PoseEstimation" << std::endl;
  }

  virtual void initialization() = 0;
  virtual void run() = 0;

private:
  CostMillisecond cost_;
};
} // namespace SensorLab
