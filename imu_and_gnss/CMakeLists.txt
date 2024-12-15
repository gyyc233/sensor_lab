file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/imu_and_gnss)

add_library(imu STATIC src/imu_integration.cc common/io_utils.cc)
target_include_directories(imu PUBLIC include)
target_link_libraries(imu ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog)

add_executable(imu_integration_test test/imu_integration_test.cc)

target_link_libraries(imu_integration_test imu glog::glog)

# INSTALL
install(
  TARGETS imu_integration_test imu
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
