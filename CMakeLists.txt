cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

# set gdb debug
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(sensor_lab LANGUAGES CXX)

# add_subdirectory(io)
add_subdirectory(algorithm)
# add_subdirectory(camera_calibration_dlt)
# add_subdirectory(backlight_adjustment)
# add_subdirectory(correctuion_2d_deformation)
