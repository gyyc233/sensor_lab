#include "DBoW3/DBoW3.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>

int main() {
  std::vector<cv::Mat> images;
  for (int i = 1; i <= 10; i++) {
    std::string path = "./data/data_tum/" + std::to_string(i) + ".png";
    images.push_back(cv::imread(path, 0));
  }

  // detect ORB features
  std::cout << "1. detecting ORB features... " << std::endl;
  cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  std::vector<cv::Mat> descriptors;
  for (cv::Mat &image : images) {
    std::vector<cv::KeyPoint> key_points;
    cv::Mat descriptor;
    detector->detectAndCompute(image, cv::Mat(), key_points, descriptor);
    descriptors.push_back(descriptor);
  }

  // create vocabulary
  //--DBoW3的简单操作
  std::cout << "2. train dictionary based on images. creating vocabulary ... "
            << std::endl;
  DBoW3::Vocabulary vocab;   // 默认构造函数 k=10,d=5
  vocab.create(descriptors); // set input
  std::cout << "vocabulary info: " << vocab << std::endl;
  vocab.save("vocabulary.yml.gz");
  std::cout << "done" << std::endl;
  return 0;
}
