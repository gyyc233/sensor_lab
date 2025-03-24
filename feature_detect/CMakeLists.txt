file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

# sudo cp libpcl_* /usr/local/lib/ to move pcl *.so
find_package(PCL 1.10 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(feature_detect STATIC ${srcs})
target_include_directories(feature_detect PUBLIC include)
target_link_libraries(feature_detect ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(orb_cv_test test/orb_cv_test.cc)
add_executable(optical_flow_test test/optical_flow_test.cc)

target_link_libraries(orb_cv_test feature_detect)
target_link_libraries(optical_flow_test feature_detect)

# INSTALL
install(
  TARGETS orb_cv_test optical_flow_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
