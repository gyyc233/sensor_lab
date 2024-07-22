file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(glog 0.6.0 REQUIRED)
find_package(gflags REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/g2o_test/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories (${gflags_INCLUDE_DIR})
include_directories (${G2O_INCLUDE_DIRS})

# add io/ path to compiler search
include_directories(${PROJECT_SOURCE_DIR}/io)
include_directories(${PROJECT_SOURCE_DIR}/common)

# add_library(my_g2o STATIC ${srcs})

# target_include_directories(my_g2o PUBLIC include)
# target_link_libraries(my_g2o ${OpenCV_LIBS} g2o_core g2o_stuffs glog::glog gflags)

add_executable(curve_fitting_test test/curve_fitting_test.cc)
target_link_libraries(curve_fitting_test ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} glog::glog gflags)
