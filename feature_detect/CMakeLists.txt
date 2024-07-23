file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(PCL 1.14.1 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(feature_detect STATIC ${srcs})
target_include_directories(feature_detect PUBLIC include)
target_link_libraries(feature_detect ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(orb_cv_test test/orb_cv_test.cc)

target_link_libraries(orb_cv_test ${PCL_LIBRARIES} ${OpenCV_LIBS})

# INSTALL
install(
  TARGETS orb_cv_test 
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
