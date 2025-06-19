file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(PCL 1.10 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build) # 改为自己的安装路径
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/pcl_test)
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/third_party)
include_directories(${PROJECT_SOURCE_DIR}/io)

add_executable(point_cloud_test test/point_cloud_test.cc)
add_executable(pcd_to_bird_eye_test test/pcd_to_bird_eye_test.cc)
add_executable(scan_to_range_image test/scan_to_range_image.cc)
add_executable(pcl_transform_tool test/pcl_transform_tool.cc)

add_library(nearest_neighbour_search STATIC 
  src/brute_force_nearest_neighbour.cc
  src/kd_tree.cc
  src/octop_tree.cc)
target_include_directories(nearest_neighbour_search PUBLIC include)
target_link_libraries(nearest_neighbour_search  ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread glog::glog gflags)

target_link_libraries(point_cloud_test ${PCL_LIBRARIES})
target_link_libraries(pcd_to_bird_eye_test ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread glog::glog gflags)
target_link_libraries(scan_to_range_image ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread glog::glog gflags)
target_link_libraries(pcl_transform_tool ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} pthread glog::glog gflags)

add_executable(nearest_neighbour_search_test test/nearest_neighbour_search_test.cc)
target_link_libraries(nearest_neighbour_search_test nearest_neighbour_search gtest)

add_executable(linear_fitting_test test/linear_fitting_test.cc)
target_link_libraries(linear_fitting_test ${PCL_LIBRARIES} ${OpenCV_LIBS} gtest glog::glog gflags)

# INSTALL
install(
  TARGETS point_cloud_test pcd_to_bird_eye_test scan_to_range_image
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
