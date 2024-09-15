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

# add_library(path_plan STATIC ${srcs})
# target_include_directories(path_plan PUBLIC include)
# target_link_libraries(path_plan ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(dijkstra_test test/dijkstra_test.cc)
target_link_libraries(dijkstra_test)

# INSTALL
install(
  TARGETS dijkstra_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
