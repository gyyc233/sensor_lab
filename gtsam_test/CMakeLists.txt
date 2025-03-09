file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package( GTSAM REQUIRED )

include_directories( ${GTSAM_INCLUDE_DIR} )
include_directories(include)
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

add_executable(localization_example test/localization_example.cc)
target_link_libraries(localization_example ${GTSAM_LIBRARIES})

add_executable(pose2_slam_example test/pose2_slam_example.cc)
target_link_libraries(pose2_slam_example ${GTSAM_LIBRARIES})

# PlanarSLAMExample
add_executable(planar_slam_example test/planar_slam_example.cc)
target_link_libraries(planar_slam_example ${GTSAM_LIBRARIES})

# vSFMexample with isam
add_executable(v_slam_with_isam_example test/v_slam_with_isam_example.cc)
target_link_libraries(v_slam_with_isam_example ${GTSAM_LIBRARIES})

# vSFMexample with isam2
add_executable(v_slam_with_isam2_example test/v_slam_with_isam2_example.cc)
target_link_libraries(v_slam_with_isam2_example ${GTSAM_LIBRARIES})
