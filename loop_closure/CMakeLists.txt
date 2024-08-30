file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cc include/*.h)

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# dbow3 
# dbow3 is a simple lib so I assume you installed it in default directory 
set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )

# add_library(loop_closure STATIC ${srcs})
# target_include_directories(loop_closure PUBLIC include)
# target_link_libraries(loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS})
