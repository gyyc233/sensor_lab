file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build) # 改为自己的安装路径
set(Pangolin_DIR /home/V01/uids0025/src/Pangolin/build)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# add io/ path to compiler search
include_directories(${PROJECT_SOURCE_DIR}/io)
include_directories(${PROJECT_SOURCE_DIR}/common)

add_library(task STATIC ${srcs})

target_include_directories(task PUBLIC include)
target_link_libraries(task ${OpenCV_LIBS})

add_executable(my_gaussian_blur test/my_gaussian_blur.cc)
add_executable(use_sophus test/use_sophus.cc)
add_executable(my_transform test/my_transform.cc)
add_executable(eigen_base_test test/eigen_base_test.cc)
add_executable(mat_base test/mat_base.cc)
add_executable(covariance_test test/covariance_test.cc)
add_executable(gauss_newton_curve_fitting_test test/gauss_newton_curve_fitting_test.cc)

target_link_libraries(my_gaussian_blur task glog::glog)
target_link_libraries(use_sophus task glog::glog ${Sophus_LIBRARIES})
target_link_libraries(my_transform task glog::glog)
target_link_libraries(eigen_base_test glog::glog)
target_link_libraries(mat_base task glog::glog)
target_link_libraries(covariance_test task glog::glog)
target_link_libraries(gauss_newton_curve_fitting_test task glog::glog)

# INSTALL
install(
  TARGETS my_gaussian_blur use_sophus
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
