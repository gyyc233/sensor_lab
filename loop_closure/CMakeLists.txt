file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dbow3 
# dbow3 is a simple lib so I assume you installed it in default directory 
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )

add_executable(feature_training test/feature_training.cc)
add_executable(loop_closure_test test/loop_closure.cc)
target_link_libraries(feature_training ${OpenCV_LIBS} ${DBoW3_LIBS})
target_link_libraries(loop_closure_test ${OpenCV_LIBS} ${DBoW3_LIBS})
