file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(PCL 1.14.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/io)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(kalman_filter STATIC ${srcs})
target_include_directories(kalman_filter PUBLIC include)
target_link_libraries(kalman_filter ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(discrete_kalman_filter_test test/discrete_kalman_filter_test.cc)

target_link_libraries(discrete_kalman_filter_test kalman_filter)

# INSTALL
install(
  TARGETS discrete_kalman_filter_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)