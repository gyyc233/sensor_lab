file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

find_package( GTSAM REQUIRED )

include_directories( ${GTSAM_INCLUDE_DIR} )
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

add_executable(localization_example test/localization_example.cc)
target_link_libraries(localization_example ${GTSAM_LIBRARIES})
