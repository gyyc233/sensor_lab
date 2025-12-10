include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/cpp_test)

# 关闭编译器优化
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-elide-constructors")

find_package(glog 0.6.0 REQUIRED)
add_executable(template_test template_test.cc)
add_executable(variadic_templates_test variadic_templates_test.cc)
add_executable(left_value_right_value_test left_value_right_value_test.cc)
add_executable(diy_smart_ptr_test diy_smart_ptr_test.cc diy_smart_ptr.hpp)
add_executable(bind_function_test bind_function_test.cc)
add_executable(write_shared_memory write_shared_memory.cc)
add_executable(read_shared_memory read_shared_memory.cc)

target_link_libraries(template_test glog::glog)

# INSTALL
install(
  TARGETS template_test variadic_templates_test left_value_right_value_test diy_smart_ptr_test
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
