file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
find_package( Boost REQUIRED )
find_package(Eigen3 REQUIRED)

include_directories( ${GTSAM_INCLUDE_DIR} )
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(localization_example test/localization_example.cc)
target_link_libraries(localization_example ${Boost_LIBRARIES} gtsam)
