include_directories(${PROJECT_SOURCE_DIR}/common)

find_package(glog 0.6.0 REQUIRED)
add_executable(template_test template_test.cc)

target_link_libraries(template_test glog::glog)

# INSTALL
install(
  TARGETS template_test 
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
