file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
# set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

add_library(imu STATIC 
  src/imu_integration.cc 
  ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc 
  src/static_imu_init.cc)
target_include_directories(imu PUBLIC include)
target_link_libraries(imu ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog)

# utm_convert.a
add_library(utm_convert STATIC ${PROJECT_SOURCE_DIR}/third_party/utm_convert/utm.cc ${PROJECT_SOURCE_DIR}/third_party/utm_convert/tranmerc.cc)
target_include_directories(utm_convert PUBLIC include)
target_link_libraries(utm_convert ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog)

add_library(gnss STATIC src/utm_convert.cc)
target_include_directories(gnss PUBLIC include)
target_link_libraries(gnss ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog utm_convert)

# imu+rtk+odom eskf
add_library(eskf STATIC include/eskf.hpp)
target_include_directories(eskf PUBLIC include)
target_link_libraries(eskf ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog imu gnss utm_convert)

# imu_preintegration
add_library(imu_preintegration STATIC src/imu_preintegration.cc)
target_include_directories(imu_preintegration PUBLIC include)
target_link_libraries(imu_preintegration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Sophus_LIBRARIES} glog::glog imu)

# imu direct integration
add_executable(imu_integration_test test/imu_integration_test.cc)
target_link_libraries(imu_integration_test imu glog::glog)

# gnss data process
add_executable(gnss_data_test test/gnss_data_test.cc)
target_link_libraries(gnss_data_test gnss glog::glog imu)

# eskf gins
add_executable(run_eskf_gins test/run_eskf_gins.cc)
target_link_libraries(run_eskf_gins eskf)

# gtest example
add_executable(gtest_preintegration test/gtest_preintegration.cc)
target_link_libraries(gtest_preintegration gtest pthread glog gflags imu_preintegration)

# INSTALL
install(
  TARGETS imu_integration_test imu
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
