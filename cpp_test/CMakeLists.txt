include_directories(${PROJECT_SOURCE_DIR}/common)

add_executable(template_test template_test.cc)

# INSTALL
install(
  TARGETS template_test 
  ARCHIVE DESTINATION ${PROJECT_SOURCE_DIR}/build/install)
