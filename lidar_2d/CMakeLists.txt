file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

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
include_directories(${PROJECT_SOURCE_DIR}/third_party)

if(ROS_CATKIN)
else()
message("lidar_2d module need `-DROS_CATKIN=ON` optional")
    return()
endif()

add_library(lidar_2d_utils STATIC src/lidar_2d_utils.cc ${PROJECT_SOURCE_DIR}/common/navigation_and_mapping/io_utils.cc)
target_include_directories(lidar_2d_utils PUBLIC include)
target_link_libraries(lidar_2d_utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags)

add_library(lidar_2d_icp STATIC src/icp_2d.cc)
target_include_directories(lidar_2d_icp PUBLIC include)
target_link_libraries(lidar_2d_icp ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags)

add_library(likelihood_field STATIC src/likelihood_field.cc)
target_include_directories(likelihood_field PUBLIC include)
target_link_libraries(likelihood_field ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} pthread glog::glog gflags ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

add_library(occupancy_map_2d STATIC src/occupancy_map.cc)
target_include_directories(occupancy_map_2d PUBLIC include)
target_link_libraries(occupancy_map_2d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags)

add_library(submap_2d STATIC src/submap.cc)
target_include_directories(submap_2d PUBLIC include)
target_link_libraries(submap_2d likelihood_field occupancy_map_2d ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags)

add_library(mapping_2d STATIC src/mapping_2d.cc)
target_include_directories(mapping_2d PUBLIC include)
target_link_libraries(mapping_2d likelihood_field occupancy_map_2d submap_2d lidar_2d_utils ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags)

add_library(multi_resolution_likelihood_field STATIC src/multi_resolution_likelihood_field.cc)
target_include_directories(multi_resolution_likelihood_field PUBLIC include)
target_link_libraries(multi_resolution_likelihood_field ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

add_library(loop_closing STATIC src/loop_closing.cc)
target_include_directories(loop_closing PUBLIC include)
target_link_libraries(loop_closing ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${catkin_LIBRARIES} glog::glog gflags ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} multi_resolution_likelihood_field submap_2d)

# TEST mr_likelihood_field_test

add_executable(2dlidar_io_test test/2dlidar_io_test.cc)
add_executable(2d_icp_s2s_test test/2d_icp_s2s_test.cc)
add_executable(likelihood_field_icp_test test/likelihood_field_icp_test.cc)
add_executable(occupancy_grid_test test/occupancy_grid_test.cc)
add_executable(mapping_2d_test test/mapping_2d_test.cc)
add_executable(mr_likelihood_field_test test/mr_likelihood_field_test.cc src/frame.cc)

target_link_libraries(2dlidar_io_test lidar_2d_utils)
target_link_libraries(2d_icp_s2s_test lidar_2d_utils lidar_2d_icp)
target_link_libraries(likelihood_field_icp_test lidar_2d_utils likelihood_field)
target_link_libraries(occupancy_grid_test lidar_2d_utils occupancy_map_2d)
target_link_libraries(mapping_2d_test lidar_2d_utils mapping_2d)
target_link_libraries(mr_likelihood_field_test lidar_2d_utils multi_resolution_likelihood_field)
