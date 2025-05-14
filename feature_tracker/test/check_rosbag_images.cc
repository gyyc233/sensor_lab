#include "feature_tracker.h"
#include "navigation_and_mapping/io_utils.h"
#include "pinhole_camera.h"
#include "sys_utils.h"
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#ifdef ROS_CATKIN
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#endif

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

  bool cv_vis = false;
  int num_of_camera = 1;

#ifdef ROS_CATKIN
  ros::Publisher pub_img, pub_match;
  ros::Publisher pub_restart;
  bool init_pub = 0;

  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
  pub_match = n.advertise<sensor_msgs::Image>("feature_img", 1000);
#endif

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

              for (unsigned int i = 0;; i++) {
                bool completed = false;
                for (int j = 0; j < num_of_camera; j++)
                  completed |= feature_tracker_ptr->updateID(
                      i); // |= 按位或运算并进行赋值

                if (!completed)
                  break;
              }

              ptr =
                  cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
              cv::Mat show_img = ptr->image;
              for (unsigned int j = 0; j < feature_tracker_ptr->cur_pts.size();
                   j++) {
                cv::circle(show_img, feature_tracker_ptr->cur_pts[j], 2,
                           cv::Scalar(255, 0, 255), 2);
              }
              if (cv_vis) {
                cv::imshow("vis", show_img);
                cv::waitKey(5);
              }

#ifdef ROS_CATKIN
              sensor_msgs::PointCloudPtr feature_points(
                  new sensor_msgs::PointCloud);
              sensor_msgs::ChannelFloat32 id_of_point;
              sensor_msgs::ChannelFloat32 u_of_point;
              sensor_msgs::ChannelFloat32 v_of_point;
              sensor_msgs::ChannelFloat32 velocity_x_of_point;
              sensor_msgs::ChannelFloat32 velocity_y_of_point;

              feature_points->header = img_msg->header;
              feature_points->header.frame_id = "world";

              std::vector<std::set<int>> hash_ids(num_of_camera);

              auto &un_pts = feature_tracker_ptr->cur_un_pts;
              auto &cur_pts = feature_tracker_ptr->cur_pts;
              auto &ids = feature_tracker_ptr->ids;
              auto &pts_velocity = feature_tracker_ptr->pts_velocity;

              for (unsigned int j = 0; j < ids.size(); j++) {
                if (feature_tracker_ptr->track_cnt[j] > 1) {
                  int p_id = ids[j];
                  hash_ids[0].insert(p_id);
                  geometry_msgs::Point32 p;
                  p.x = un_pts[j].x;
                  p.y = un_pts[j].y;
                  p.z = 1;

                  feature_points->points.push_back(p);
                  id_of_point.values.push_back(p_id * num_of_camera + 0);
                  u_of_point.values.push_back(cur_pts[j].x);
                  v_of_point.values.push_back(cur_pts[j].y);
                  velocity_x_of_point.values.push_back(pts_velocity[j].x);
                  velocity_y_of_point.values.push_back(pts_velocity[j].y);

                  std::cout << "p: " << p << std::endl;
                  std::cout << "p_id * num_of_camera + 0: "
                            << p_id * num_of_camera + 0 << std::endl;
                  std::cout << "u_of_point: " << cur_pts[j].x << std::endl;
                  std::cout << "u_of_point: " << cur_pts[j].y << std::endl;
                  std::cout << "velocity_x_of_point: " << pts_velocity[j].x
                            << std::endl;
                  std::cout << "velocity_x_of_point: " << pts_velocity[j].y
                            << std::endl;
                }
              }

              feature_points->channels.push_back(id_of_point);
              feature_points->channels.push_back(u_of_point);
              feature_points->channels.push_back(v_of_point);
              feature_points->channels.push_back(velocity_x_of_point);
              feature_points->channels.push_back(velocity_y_of_point);

              if (!init_pub) {
                init_pub = 1;
              } else
                pub_img.publish(feature_points);

              pub_match.publish(ptr->toImageMsg());

#endif
            }
            return true;
          })
      .Go();
#ifdef ROS_CATKIN
  ros::spin();
#endif
  return 0;
}
