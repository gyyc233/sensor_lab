#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

DEFINE_string(pcd_path, "./data/pcd/map_example.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, 0.2, "俯视图最低高度");
DEFINE_double(max_z, 2.5, "俯视图最高高度");

/// @brief point cloud build bird eye view image
/// @param cloud point cloud
void GenerateBEVImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // calculate points cloud bound
  auto minmax_x =
      std::minmax_element(cloud->points.begin(), cloud->points.end(),
                          [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
                            return p1.x < p2.x;
                          });
  auto minmax_y =
      std::minmax_element(cloud->points.begin(), cloud->points.end(),
                          [](const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
                            return p1.y < p2.y;
                          });
  double min_x = minmax_x.first->x;
  double max_x = minmax_x.second->x;
  double min_y = minmax_y.first->y;
  double max_y = minmax_y.second->y;

  // set 2D image property
  const double inv_r = 1.0 / FLAGS_image_resolution;

  const int image_rows = int((max_y - min_y) * inv_r);
  const int image_cols = int((max_x - min_x) * inv_r);

  float x_center = 0.5 * (max_x + min_x);
  float y_center = 0.5 * (max_y + min_y);
  float x_center_image = image_cols / 2;
  float y_center_image = image_rows / 2;

  // 生成图像
  cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

  for (const auto &pt : cloud->points) {
    int x = int((pt.x - x_center) * inv_r + x_center_image);
    int y = int((pt.y - y_center) * inv_r + y_center_image);
    if (x < 0 || x >= image_cols || y < 0 || y >= image_rows ||
        pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
      continue;
    }

    image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
  }

  cv::imwrite("./bev.png", image);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_pcd_path.empty()) {
    LOG(ERROR) << "pcd path is empty";
    return -1;
  }

  // 读取点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

  if (cloud->empty()) {
    LOG(ERROR) << "cannot load cloud file";
    return -1;
  }

  LOG(INFO) << "cloud points: " << cloud->size();
  GenerateBEVImage(cloud);

  return 0;
}
