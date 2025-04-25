file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

# sudo cp libpcl_* /usr/local/lib/ to move pcl *.so
find_package(PCL 1.10 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message(STATUS "Sophus_INCLUDE_DIRS was: ${Sophus_INCLUDE_DIRS}")
message(STATUS "Sophus_LIBRARIES was: ${Sophus_LIBRARIES}")

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
link_directories(yaml-cpp)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/algorithm/include)

if(ROS_CATKIN)
else()
message("lidar_3d module need `-DROS_CATKIN=ON` optional")
    return()
endif()

# add_library(feature_tracker STATIC ${srcs})
# target_include_directories(feature_tracker PUBLIC include)
# target_link_libraries(feature_tracker ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(check_rosbag_images test/check_rosbag_images.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_link_libraries(check_rosbag_images ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags yaml-cpp algorithm)

# INSTALL
install(
  TARGETS check_rosbag_images
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
