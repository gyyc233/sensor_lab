#include "gaussian_blur/gaussian_blur.h"
#include <glog/logging.h>
#include <memory>

int main() {
  std::unique_ptr<GaussianBlur> gaussian_blur_ptr =
      std::make_unique<GaussianBlur>();
  gaussian_blur_ptr->builtGaussianKernel(3, 0.5);

  return 0;
}
