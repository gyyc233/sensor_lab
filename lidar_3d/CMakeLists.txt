file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-fopenmp")
set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
# set(Sophus_DIR /home/bot/src/Sophus/build) # 改为自己的安装路径

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message(STATUS "Sophus_INCLUDE_DIRS was: ${Sophus_INCLUDE_DIRS}")
message(STATUS "Sophus_LIBRARIES was: ${Sophus_LIBRARIES}")

# include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lidar_2d/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories (${G2O_INCLUDE_DIRS})
# g2o

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
link_directories(yaml-cpp)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/pcl_test)
include_directories(${PROJECT_SOURCE_DIR}/third_party)
include_directories(${PROJECT_SOURCE_DIR}/imu_and_gnss)
include_directories(${PROJECT_SOURCE_DIR}/lidar_3d)

if(ROS_CATKIN)
else()
message("lidar_3d module need `-DROS_CATKIN=ON` optional")
    return()
endif()

# icp 3d
add_library(icp_3d STATIC src/icp_3d.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(icp_3d PUBLIC include)
target_link_libraries(icp_3d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags nearest_neighbour_search)

# ndt 3d
add_library(ndt_3d STATIC src/ndt_3d.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(ndt_3d PUBLIC include)
target_link_libraries(ndt_3d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags nearest_neighbour_search)

# direct ndt
add_library(direct_ndt_lo STATIC src/direct_ndt_lo.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(direct_ndt_lo PUBLIC include)
target_link_libraries(direct_ndt_lo ndt_3d)

# incremental ndt
add_library(incremental_ndt_lo STATIC src/incremental_ndt_lo.cc src/incremental_ndt.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(incremental_ndt_lo PUBLIC include)
target_link_libraries(incremental_ndt_lo ndt_3d ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

# detect corners and planes
add_library(feature_extraction STATIC src/feature_extraction.cc)
target_include_directories(feature_extraction PUBLIC include)
target_link_libraries(feature_extraction ${PCL_LIBRARIES} glog::glog)

# like loam
add_library(loam_like_odom STATIC src/loam_like_odom.cc)
target_include_directories(loam_like_odom PUBLIC include)
target_link_libraries(loam_like_odom feature_extraction icp_3d nearest_neighbour_search)

# loosely coupled lio
add_library(loosely_lio STATIC
    loosely_coupled_lio/loosely_lio.cc
    loosely_coupled_lio/measure_sync.cc 
    loosely_coupled_lio/cloud_convert.cc)
target_include_directories(loosely_lio PUBLIC include)
target_link_libraries(loosely_lio ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags eskf imu incremental_ndt_lo yaml-cpp)

# tightly coupled iekf lio
add_library(iekf_lio STATIC
    iekf/iekf.hpp
    iekf/iekf_lio.cc)
target_include_directories(iekf_lio PUBLIC include)
target_link_libraries(iekf_lio ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags loosely_lio eskf imu incremental_ndt_lo yaml-cpp)

# executable
add_executable(icp_3d_test test/icp_3d_test.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc ${PROJECT_SOURCE_DIR}/pcl_test/src/kd_tree.cc)
target_link_libraries(icp_3d_test icp_3d ndt_3d)

add_executable(run_gen_simu_data test/run_gen_simu_data.cc src/gen_simu_data.cc)
target_link_libraries(run_gen_simu_data icp_3d)


add_executable(direct_ndt_lo_test test/direct_ndt_lo_test.cc)
target_link_libraries(direct_ndt_lo_test direct_ndt_lo)

add_executable(incremental_ndt_lo_test test/incremental_ndt_lo_test.cc)
target_link_libraries(incremental_ndt_lo_test incremental_ndt_lo)

add_executable(loam_like_lo_test test/loam_like_lo_test.cc ${PROJECT_SOURCE_DIR}/pcl_test/src/kd_tree.cc)
target_link_libraries(loam_like_lo_test loam_like_odom  icp_3d)

add_executable(loosely_lio_test test/loosely_lio_test.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc ${PROJECT_SOURCE_DIR}/pcl_test/src/kd_tree.cc)
target_link_libraries(loosely_lio_test loosely_lio yaml-cpp)

add_executable(iekf_lio_test test/iekf_lio_test.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc ${PROJECT_SOURCE_DIR}/pcl_test/src/kd_tree.cc)
target_link_libraries(iekf_lio_test iekf_lio yaml-cpp)
