file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(gflags REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
set(Pangolin_DIR /home/V01/uids0025/src/Pangolin/build)
set(Ceres_DIR /home/V01/uids0025/src/ceres-solver-2.2.0/build)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories (${gflags_INCLUDE_DIR})

# add io/ path to compiler search
include_directories(${PROJECT_SOURCE_DIR}/io)
include_directories(${PROJECT_SOURCE_DIR}/common)

# add_library(my_cere STATIC ${srcs})

# target_include_directories(my_cere PUBLIC include)
# target_link_libraries(my_cere ${OpenCV_LIBS} Ceres::ceres glog::glog)

add_executable(cere_linear_test test/cere_linear_test.cc)
add_executable(powell test/powell.cc)

target_link_libraries(cere_linear_test ${OpenCV_LIBS} Ceres::ceres glog::glog)
target_link_libraries(powell Ceres::ceres glog::glog gflags)
