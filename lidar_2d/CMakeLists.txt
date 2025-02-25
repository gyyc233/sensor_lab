file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
# set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
# set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

if(ROS_CATKIN)
else()
message("lidar_2d module need `-DROS_CATKIN=ON` optional")
    return()
endif()

add_library(lidar_2d_utils STATIC src/lidar_2d_utils.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(lidar_2d_utils PUBLIC include)
target_link_libraries(lidar_2d_utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags)

add_executable(2dlidar_io_test test/2dlidar_io_test.cc)

target_link_libraries(2dlidar_io_test lidar_2d_utils)
