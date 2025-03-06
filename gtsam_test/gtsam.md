
# GTSAM

- Georgia Tech Smoothing and Mapping Library
- GTSAM is a C++ library that implements smoothing and mapping (SAM) in robotics and vision, using Factor Graphs and Bayes Networks as the underlying computing paradigm rather than sparse matrices.

## install gtsam

```bash
git clone git@github.com:borglab/gtsam.git
cd gtsam
git checkout 4.2.0
mkdir build && cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j4
sudo make install
```

- `-DGTSAM_USE_SYSTEM_EIGEN=ON` avoid Eigen version conflicts
- copy `libmetis-gtsam.so` to `/usr/lib/`
-  `sudo cp /usr/local/lib/libmetis-gtsam.so /usr/lib` [libmetis-gtsam issue 380](https://github.com/borglab/gtsam/issues/380)

set CMakeLists.txt
```cmake
# gtsam 
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )

#......
target_link_libraries(gtsam)
```

### my gtsam build

```shell
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V42=OFF -DGTSAM_USE_QUATERNIONS=OFF -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_SINGLE_TEST_EXE=ON -DBOOST_ROOT=$BOOST_ROOT -DBoost_NO_SYSTEM_PATHS=OFF -DBoost_ARCHITECTURE=-x64 -DEigen3_DIR=/usr/include/eigen3 ..
```

### uninstall

```shell
cd build
build$ sudo xargs rm -rf < install_manifest.txt
```

## 参考文章

- [GTSAM快速入门](https://zhuanlan.zhihu.com/p/621999120)
