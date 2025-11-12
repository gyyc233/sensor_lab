cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

# set gdb debug
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(sensor_lab LANGUAGES CXX)

# for basic CI/CD
option(ONLY_CPP_TEST "Only Cpp test" OFF)
if(ONLY_CPP_TEST)
  message(STATUS "Only Cpp test is ON")
  # add_subdirectory(algorithm)
  add_subdirectory(cpp_test)
else()
  add_subdirectory(algorithm)
  add_subdirectory(feature_detect)
  add_subdirectory(pose_estimation)
  add_subdirectory(cpp_test)
  add_subdirectory(pcl_test)
  add_subdirectory(cere_test)
  add_subdirectory(g2o_test)
  add_subdirectory(task)
  add_subdirectory(loop_closure)
  add_subdirectory(dense_mapping)
  add_subdirectory(kalman_filter)
  add_subdirectory(path_plan)
  add_subdirectory(path_tracking)
  add_subdirectory(imu_and_gnss)
  add_subdirectory(gtsam_test)
endif()

# =============== add ROS =====================
option(ROS_CATKIN "ros catkin" OFF)

if(ROS_CATKIN)
  message(STATUS "ros catkin was ON")
  add_definitions(-DROS_CATKIN)
  include(cmake/find_ros.cmake)
  include(cmake/third_party.cmake)
  add_subdirectory(lidar_2d)
  add_subdirectory(lidar_3d)
  add_subdirectory(feature_tracker)
endif()
# =============== add ROS =====================

# =============== cuda =====================
option(CUDA_ENABLE "cuda enable" OFF)

if(CUDA_ENABLE)
  message(STATUS "cuda enable was ON")
endif()
# =============== cuda =====================

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# copy floder to install path
set(build_data_include ${CMAKE_CURRENT_SOURCE_DIR}/build)
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory  ${CMAKE_CURRENT_SOURCE_DIR}/data ${build_data_include}/data)
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/include/common ${build_data_include}/algorithm/include/common)



