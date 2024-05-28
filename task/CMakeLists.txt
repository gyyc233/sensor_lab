file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(task STATIC ${srcs})

target_include_directories(task PUBLIC include)
target_link_libraries(task ${OpenCV_LIBS} glog::glog)

add_executable(my_gaussian_blur test/my_gaussian_blur.cc)

target_link_libraries(my_gaussian_blur task glog::glog)

# INSTALL
install(
  TARGETS my_gaussian_blur
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
