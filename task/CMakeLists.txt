file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build) # 改为自己的安装路径
find_package( Sophus REQUIRED )
include_directories(${Sophus_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# add io/ path to compiler search
include_directories(${PROJECT_SOURCE_DIR}/io)

add_library(task STATIC ${srcs})

target_include_directories(task PUBLIC include)
target_link_libraries(task ${OpenCV_LIBS} glog::glog)

add_executable(my_gaussian_blur test/my_gaussian_blur.cc)
add_executable(use_sophus test/use_sophus.cc)
add_executable(my_transform test/my_transform.cc)

target_link_libraries(my_gaussian_blur task glog::glog)
target_link_libraries(use_sophus task glog::glog ${Sophus_LIBRARIES})
target_link_libraries(my_transform task glog::glog)

# INSTALL
install(
  TARGETS my_gaussian_blur use_sophus
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
