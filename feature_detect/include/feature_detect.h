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
};
} // namespace SensorLab
