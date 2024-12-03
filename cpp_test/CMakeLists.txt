include_directories(${PROJECT_SOURCE_DIR}/common)

# 关闭编译器优化
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-elide-constructors")

find_package(glog 0.6.0 REQUIRED)
add_executable(template_test template_test.cc)
add_executable(variadic_templates_test variadic_templates_test.cc)
add_executable(left_value_right_value_test left_value_right_value_test.cc)

target_link_libraries(template_test glog::glog)

# INSTALL
install(
  TARGETS template_test variadic_templates_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
