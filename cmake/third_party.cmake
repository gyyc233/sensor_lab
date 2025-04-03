# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
message(STATUS "CMAKE_MODULE_PATH was: ${CMAKE_MODULE_PATH}")

# livox ros driver
add_subdirectory(third_party/livox_ros_driver)

include_directories(${CMAKE_BINARY_DIR}/devel/include) # 引用ros生成的msg header
message(STATUS "CMAKE_BINARY_DIR was: ${CMAKE_BINARY_DIR}")
message(STATUS "need `source ~/.bashrc`")
