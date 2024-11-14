file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/io)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ================================================================
# add matplotlib-cpp
add_library(path_tracking_matplotlib_cpp INTERFACE)
target_include_directories(path_tracking_matplotlib_cpp INTERFACE ${PROJECT_SOURCE_DIR}/common)
target_compile_features(path_tracking_matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(path_tracking_matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(path_tracking_matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(path_tracking_matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS path_tracking_matplotlib_cpp
  EXPORT install_targets
)
# ================================================================
add_library(path_tracking STATIC ${srcs})
target_include_directories(path_tracking PUBLIC include)
target_link_libraries(path_tracking ${OpenCV_LIBS} ${PCL_LIBRARIES} path_tracking_matplotlib_cpp)

add_executable(pid_controller_test test/pid_controller_test.cc utils/kinematic_model.cc utils/my_reference_path.cc)
target_include_directories(pid_controller_test PUBLIC utils)
target_link_libraries(pid_controller_test path_tracking)

add_executable(pure_pursuit_test test/pure_pursuit_test.cc utils/kinematic_model.cc)
target_include_directories(pure_pursuit_test PUBLIC utils)
target_link_libraries(pure_pursuit_test path_tracking)

add_executable(stanley_test test/stanley_test.cc utils/kinematic_model.cc)
target_include_directories(stanley_test PUBLIC utils)
target_link_libraries(stanley_test path_tracking)

add_executable(lqr_control_test test/lqr_control_test.cc utils/kinematic_model.cc utils/my_reference_path.cc)
target_include_directories(lqr_control_test PUBLIC utils)
target_link_libraries(lqr_control_test path_tracking)

# INSTALL
install(
  TARGETS pid_controller_test pure_pursuit_test stanley_test lqr_control_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
