
## install gtsam

```bash
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout 4.2.0
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

set CMakeLists.txt
```cmake
# gtsam 
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )

#......
target_link_libraries(gtsam)
```
