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

include_directories(${PROJECT_SOURCE_DIR}/pcl_test)
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

add_executable(point_cloud_test test/point_cloud_test.cc)
add_executable(pcd_to_bird_eye_test test/pcd_to_bird_eye_test.cc)
add_executable(scan_to_range_image test/scan_to_range_image.cc)

target_link_libraries(point_cloud_test ${PCL_LIBRARIES})
target_link_libraries(pcd_to_bird_eye_test ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread glog::glog gflags)
target_link_libraries(scan_to_range_image ${PCL_LIBRARIES} ${OpenCV_LIBS} pthread glog::glog gflags)

# INSTALL
install(
  TARGETS point_cloud_test pcd_to_bird_eye_test scan_to_range_image
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
