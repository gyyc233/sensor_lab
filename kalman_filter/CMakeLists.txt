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

# ================================================================
# add matplotlib-cpp
add_library(kalman_matplotlib_cpp INTERFACE)
target_include_directories(kalman_matplotlib_cpp INTERFACE ${PROJECT_SOURCE_DIR}/common)
target_compile_features(kalman_matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(kalman_matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(kalman_matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(kalman_matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS kalman_matplotlib_cpp
  EXPORT install_targets
)
# ================================================================

add_executable(discrete_kalman_filter_test test/discrete_kalman_filter_test.cc)
add_executable(range_sensor_ekf_sample_test test/range_sensor_ekf_sample_test.cc)
target_link_libraries(discrete_kalman_filter_test kalman_filter kalman_matplotlib_cpp)
target_link_libraries(range_sensor_ekf_sample_test kalman_filter kalman_matplotlib_cpp)

# INSTALL
install(
  TARGETS discrete_kalman_filter_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)