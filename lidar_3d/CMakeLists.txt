file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
# set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message(STATUS "Sophus_INCLUDE_DIRS was: ${Sophus_INCLUDE_DIRS}")
message(STATUS "Sophus_LIBRARIES was: ${Sophus_LIBRARIES}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lidar_2d/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories (${G2O_INCLUDE_DIRS})
# g2o

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

if(ROS_CATKIN)
else()
message("lidar_3d module need `-DROS_CATKIN=ON` optional")
    return()
endif()