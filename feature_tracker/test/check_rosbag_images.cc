#include "navigation_and_mapping/io_utils.h"
#include "sys_utils.h"
#include "undistortion/undistortion.h"
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

// http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag
DEFINE_string(bag_path, "./data/feature_tracker/V2_01_easy.bag",
              "path to rosbag");
DEFINE_string(topic, "/cam0/image_raw", "topic of left camera");
DEFINE_string(dataset_type, "IMAGES", "image rosbag"); // 数据集类型

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path,
                          sad::Str2DatasetType(FLAGS_dataset_type));

  std::unique_ptr<Algorithm::Undistortion> undistortion_ptr =
      std::make_unique<Algorithm::Undistortion>();

  std::vector<double> camera_intrinsics = {4.616e+02, 4.603e+02, 3.630e+02,
                                           2.481e+02};
  std::vector<double> distortion_params = {-2.917e-01, 8.228e-02, 0, 5.333e-05,
                                           -1.578e-04};

  rosbag_io
      .AddImageHandle(
          FLAGS_topic,
          [&](const sensor_msgs::Image::Ptr &img_msg) -> bool {
            if (img_msg != nullptr) {
              cv_bridge::CvImageConstPtr ptr;
              if (img_msg->encoding == "8UC1") {
                sensor_msgs::Image img;
                img.header = img_msg->header;
                img.height = img_msg->height;
                img.width = img_msg->width;
                img.is_bigendian = img_msg->is_bigendian;
                img.step = img_msg->step;
                img.data = img_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img,
                                          sensor_msgs::image_encodings::MONO8);
              } else
                ptr = cv_bridge::toCvCopy(img_msg,
                                          sensor_msgs::image_encodings::MONO8);

              cv::Mat show_img = ptr->image;
              undistortion_ptr->inputParams(show_img, camera_intrinsics,
                                            distortion_params);
              undistortion_ptr->run();
              cv::imshow("undistrotion", undistortion_ptr->getOutput());
              cv::waitKey(30);
            }
            return true;
          })
      .Go();

  return 0;
}
