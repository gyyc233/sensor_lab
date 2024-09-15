file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(Sophus_DIR /home/V01/uids0025/src/Sophus/build)
find_package(Sophus REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

# pcl 
find_package(PCL 1.10 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

# octomap 
find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )

include_directories(${PROJECT_SOURCE_DIR}/io)
include_directories(${PROJECT_SOURCE_DIR}/common)

add_executable(mono_dense_mapping test/mono_dense_mapping.cc)
add_executable(rgbd_dense_mapping test/rgbd_dense_mapping.cc)

target_link_libraries(mono_dense_mapping ${OpenCV_LIBS} ${Sophus_LIBRARIES})
target_link_libraries(rgbd_dense_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

# INSTALL
install(
  TARGETS mono_dense_mapping rgbd_dense_mapping
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
