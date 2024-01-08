file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/log)

add_library(algorithm STATIC ${srcs})

# PUBLIC 共享algorithm/include 这个头文件路径
target_include_directories(algorithm PUBLIC include)
target_link_libraries(algorithm ${OpenCV_LIBS})

add_executable(cubic_spline_test test/cubic_spline_test.cc)
target_link_libraries(cubic_spline_test algorithm)

# INSTALL
install(
  TARGETS cubic_spline_test algorithm
  RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/build/install
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)