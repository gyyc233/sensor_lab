# ros
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(${catkin_INCLUDE_DIRS})
message(STATUS "catkin_INCLUDE_DIRS was: ${catkin_INCLUDE_DIRS}")
