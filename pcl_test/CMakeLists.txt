file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(PCL 1.10 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_test point_cloud_test.cc)

target_link_libraries(point_cloud_test ${PCL_LIBRARIES})

# INSTALL
install(
  TARGETS point_cloud_test 
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
