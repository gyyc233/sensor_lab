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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lidar_2d/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories (${G2O_INCLUDE_DIRS})
# g2o

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/pcl_test)
include_directories(${PROJECT_SOURCE_DIR}/third_party)

if(ROS_CATKIN)
else()
message("lidar_3d module need `-DROS_CATKIN=ON` optional")
    return()
endif()

add_library(icp_3d STATIC src/icp_3d.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(icp_3d PUBLIC include)
target_link_libraries(icp_3d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags nearest_neighbour_search)

add_library(ndt_3d STATIC src/ndt_3d.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(ndt_3d PUBLIC include)
target_link_libraries(ndt_3d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags nearest_neighbour_search)

add_library(direct_ndt_lo STATIC src/direct_ndt_lo.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(direct_ndt_lo PUBLIC include)
target_link_libraries(direct_ndt_lo ndt_3d)

add_library(feature_extraction STATIC src/feature_extraction.cc)
target_include_directories(feature_extraction PUBLIC include)
target_link_libraries(feature_extraction ${PCL_LIBRARIES} glog::glog)

add_library(loam_like_odom STATIC src/loam_like_odom.cc)
target_include_directories(loam_like_odom PUBLIC include)
target_link_libraries(loam_like_odom feature_extraction icp_3d)

add_executable(icp_3d_test test/icp_3d_test.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc ${PROJECT_SOURCE_DIR}/pcl_test/src/kd_tree.cc)
target_link_libraries(icp_3d_test icp_3d ndt_3d)

add_executable(run_gen_simu_data test/run_gen_simu_data.cc src/gen_simu_data.cc)
target_link_libraries(run_gen_simu_data icp_3d)


add_executable(direct_ndt_lo_test test/direct_ndt_lo_test.cc)
target_link_libraries(direct_ndt_lo_test direct_ndt_lo)

add_executable(loam_like_lo_test test/loam_like_lo_test.cc)
target_link_libraries(loam_like_lo_test loam_like_odom)
