# ros
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        rosbag
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        geometry_msgs
        )
include_directories(${catkin_INCLUDE_DIRS})
message(STATUS "catkin_INCLUDE_DIRS was: ${catkin_INCLUDE_DIRS}")
