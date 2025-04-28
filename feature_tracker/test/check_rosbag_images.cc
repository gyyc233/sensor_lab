#include "feature_tracker.h"
#include "navigation_and_mapping/io_utils.h"
#include "pinhole_camera.h"
#include "sys_utils.h"
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

// http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag

int main(int argc, char **argv) {
  std::string bag_path = "./data/feature_tracker/V2_01_easy.bag";
  std::string topic = "/cam0/image_raw";
  std::string dataset_type = "IMAGES";

  sad::RosbagIO rosbag_io(bag_path, sad::Str2DatasetType(dataset_type));

  // [fx,fy,cx,cy,focal]
  std::vector<double> camera_intrinsics = {4.616e+02, 4.603e+02, 3.630e+02,
                                           2.481e+02, 460.0};
  // [k1,k2,k3,p1,p2]
  std::vector<double> distortion_params = {-2.917e-01, 8.228e-02, 0, 5.333e-05,
                                           -1.578e-04};

  sensor_lab::PinholeCamera::Parameters camera_params(
      topic, 752, 480, distortion_params[0], distortion_params[1],
      distortion_params[3], distortion_params[4], camera_intrinsics[0],
      camera_intrinsics[1], camera_intrinsics[2], camera_intrinsics[3],
      camera_intrinsics[4]);
  std::cout << camera_params << std::endl;

  sensor_lab::PinholeCameraPtr camera =
      std::make_shared<sensor_lab::PinholeCamera>(camera_params);

  std::unique_ptr<sensor_lab::FeatureTracker> feature_tracker_ptr =
      std::make_unique<sensor_lab::FeatureTracker>();

  feature_tracker_ptr->setCamera(camera);

  rosbag_io
      .AddImageHandle(
          topic,
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

              feature_tracker_ptr->readImage(ptr->image,
                                             img_msg->header.stamp.toSec());

              // vis
              ptr =
                  cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
              cv::Mat show_img = ptr->image;
              for (unsigned int j = 0; j < feature_tracker_ptr->cur_pts.size();
                   j++) {
                cv::circle(show_img, feature_tracker_ptr->cur_pts[j], 2,
                           cv::Scalar(255, 0, 255), 2);
              }

              cv::imshow("vis", show_img);
              cv::waitKey(5);
            }
            return true;
          })
      .Go();

  return 0;
}
