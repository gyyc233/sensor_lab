#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

/// @brief left top and right below coordination
struct AABB_BOX {
  int x; // 矩形左上角x像素坐标
  int y; // 矩形左上角y像素坐标
  int width;
  int height;
  float score; // confidence

  AABB_BOX(const cv::Rect &rect, float score)
      : x(rect.x), y(rect.y), width(rect.width), height(rect.height),
        score(score) {}

  cv::Rect AABB_BOX2rect() { return cv::Rect(x, y, width, height); }

  friend std::ostream &operator<<(std::ostream &out, const AABB_BOX &params) {
    out << " [" << params.x << ", " << params.y << ", " << params.width << ", "
        << params.height << ", " << params.score << "]" << std::endl;
    return out;
  }
};

float iou(AABB_BOX &box_1, AABB_BOX &box_2) {
  // 交集部分左上角x,y像素
  double max_x = std::max(box_1.x, box_2.x);
  double max_y = std::max(box_1.y, box_2.y);

  // 交集部分右下角x,y像素
  double min_x = std::min(box_1.x + box_1.width, box_2.x + box_2.width);
  double min_y = std::min(box_1.y + box_1.height, box_2.y + box_2.height);

  if (min_x <= max_x || min_y <= max_y) {
    return 0;
  }

  // 计算交集面积
  float over_area = (min_x - max_x) * (min_y - max_y);

  // 计算并集面积
  float area_a = box_1.width * box_1.height;
  float area_b = box_2.width * box_2.height;
  float iou = over_area / (area_a + area_b - over_area);

  return iou;
}

bool compare(AABB_BOX &box_1, AABB_BOX &box_2) {
  return (box_1.score > box_2.score);
}

/// @brief non maximum suppression
/// @param vec_boxes aabb boxes
/// @param threshold iou threshold
/// @return nms boxes
std::vector<AABB_BOX> nms(std::vector<AABB_BOX> &vec_boxes, float threshold) {
  std::vector<AABB_BOX> res;

  while (vec_boxes.size() > 0) {
    std::sort(vec_boxes.begin(), vec_boxes.end(), compare);
    res.push_back(vec_boxes[0]);

    for (auto it = vec_boxes.begin() + 1; it != vec_boxes.end();) {
      float iou_value = iou(*vec_boxes.begin(), *(it));
      if (iou_value > threshold) {
        it = vec_boxes.erase(it);
      } else {
        it++;
      }
    }

    vec_boxes.erase(vec_boxes.begin());
  }

  return res;
}

int main(int argc, char *argv[]) {
  std::vector<cv::Rect> src_rects;
  std::vector<float> scores;

  src_rects.emplace_back(cv::Point(10, 25), cv::Point(70, 80));
  scores.emplace_back(0.8f);

  src_rects.emplace_back(cv::Point(12, 30), cv::Point(76, 94));
  scores.emplace_back(0.7f);

  src_rects.emplace_back(cv::Point(12, 36), cv::Point(76, 110));
  scores.emplace_back(0.5f);

  src_rects.emplace_back(cv::Point(72, 36), cv::Point(200, 164));
  scores.emplace_back(0.3f);

  src_rects.emplace_back(cv::Point(84, 58), cv::Point(212, 186));
  scores.emplace_back(0.4f);

  src_rects.emplace_back(cv::Point(90, 50), cv::Point(220, 190));
  scores.emplace_back(0.45f);

  std::vector<AABB_BOX> aabb_boxes;
  for (size_t i = 0; i < src_rects.size(); i++) {
    aabb_boxes.push_back(AABB_BOX(src_rects[i], scores[i]));
    std::cout << aabb_boxes[i];
  }

  cv::Size size(0, 0);
  for (const auto &r : src_rects) {
    size.width = std::max(size.width, r.x + r.width);
    size.height = std::max(size.height, r.y + r.height);
  }

  cv::Mat img =
      cv::Mat(2 * size.height, 2 * size.width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat img_copy = img.clone();

  for (size_t i = 0; i < src_rects.size(); ++i) {
    const auto &r = src_rects[i];

    cv::rectangle(img, r, cv::Scalar(0, 0, 255), 2);
    cv::putText(img, std::to_string(scores[i]),
                cv::Point(r.x + 2, r.y + r.height - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1,
                cv::LINE_8, false);
  }

  cv::namedWindow("before", cv::WINDOW_AUTOSIZE);
  cv::imshow("before", img);
  cv::waitKey(1);

  std::vector<AABB_BOX> nms_result;
  nms_result = nms(aabb_boxes, 0.1);

  for (auto r : nms_result) {
    cv::rectangle(img_copy, r.AABB_BOX2rect(), cv::Scalar(0, 255, 0), 2);
  }

  cv::namedWindow("after", cv::WINDOW_AUTOSIZE);
  cv::imshow("after", img_copy);

  cv::waitKey(0);

  return 0;
}
