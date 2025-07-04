file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(algorithm STATIC ${srcs})

# PUBLIC 共享algorithm/include 这个头文件路径
target_include_directories(algorithm PUBLIC include)
target_link_libraries(algorithm ${OpenCV_LIBS} glog::glog)

include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/io)

# ================================================================
# add matplotlib-cpp
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp INTERFACE ${PROJECT_SOURCE_DIR}/common)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)
# ================================================================


add_executable(cubic_spline_test test/cubic_spline_test.cc)
add_executable(dlt_camera_calibration_test test/dlt_camera_calibration_test.cc)
add_executable(moving_lest_squares_test test/moving_lest_squares_test.cc)
add_executable(least_square_polynomial_curve_fit_test test/least_square_polynomial_curve_fit_test.cc)
add_executable(undistortion test/undistortion_test.cc)
add_executable(tsai_lenz_eye_in_hand test/tsai_lenz_eye_in_hand.cc)
add_executable(mahalanobis_distance_test test/mahalanobis_distance_test.cc)
add_executable(cv_gl_projection_test test/cv_gl_projection_test.cc)
add_executable(tps_test test/tps_test.cc)
add_executable(bilinear_interpol_test test/bilinear_interpol_test.cc)
add_executable(non_maximun_suppression_test test/non_maximun_suppression_test.cc)

target_link_libraries(cubic_spline_test algorithm glog::glog)
target_link_libraries(dlt_camera_calibration_test algorithm glog::glog)
target_link_libraries(moving_lest_squares_test algorithm glog::glog matplotlib_cpp)
target_link_libraries(least_square_polynomial_curve_fit_test algorithm glog::glog matplotlib_cpp)
target_link_libraries(undistortion algorithm glog::glog matplotlib_cpp)
target_link_libraries(tsai_lenz_eye_in_hand algorithm glog::glog matplotlib_cpp)
target_link_libraries(mahalanobis_distance_test algorithm glog::glog)
target_link_libraries(cv_gl_projection_test algorithm)
target_link_libraries(tps_test algorithm)
target_link_libraries(bilinear_interpol_test algorithm glog::glog matplotlib_cpp)
target_link_libraries(non_maximun_suppression_test ${OpenCV_LIBS})

# INSTALL
install(
  TARGETS cubic_spline_test dlt_camera_calibration_test moving_lest_squares_test algorithm
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)

# copy headers files to install path
install(FILES
  "${PROJECT_SOURCE_DIR}/algorithm/include/common/matplotlibcpp.h"
  DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
  