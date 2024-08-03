#ifndef __SENSORLAB_FEATURE_H__
#define __SENSORLAB_FEATURE_H__

#include "my_time.h"
#include <iostream>

namespace SensorLab {
class FeatureDetect {
public:
  FeatureDetect() { std::cout << "construct FeatureDetect" << std::endl; }
  virtual ~FeatureDetect() {
    std::cout << "destruct FeatureDetect" << std::endl;
  }

  virtual void initialization() = 0;
  virtual void run() = 0;

private:
  CostMillisecond cost_;
};
} // namespace SensorLab

#endif
