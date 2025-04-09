- [GTSAM](#gtsam)
  - [install gtsam](#install-gtsam)
    - [my gtsam build](#my-gtsam-build)
    - [visualization](#visualization)
    - [uninstall](#uninstall)
  - [参考文章](#参考文章)

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
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=ON -DGTSAM_ALLOW_DEPRECATED_SINCE_V42=OFF -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_USE_SYSTEM_METIS=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_SINGLE_TEST_EXE=ON -DBOOST_ROOT=$BOOST_ROOT -DBoost_NO_SYSTEM_PATHS=OFF -DBoost_ARCHITECTURE=-x64 -DEigen3_DIR=/usr/include/eigen3 ..

make

sudo make install

sudo cp /usr/local/lib/libmetis-gtsam.so /usr/lib
```

- 在我的机器上gtsam 4.2.0可以编译，但是使用时崩溃，试了很多办法，幸运的是，最后我使用上述编译选项终于过了，它们来自`.github/scripts/unix.sh`
- `{CMAKE_CXX_FLAGS}`设置`set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")` 设立删除了`-mfma`(原本用于加速Eigen计算),否则gtsam会崩溃
- `-march=native`原本用于CPU优化代码，但是会影响程序的可移植性，所依赖的项目也要使用这个标志，否则会导致gtsam出现segfault, 这里disable
- Eigen版本问题，单独使用`-DGTSAM_USE_SYSTEM_EIGEN=ON`并不会把gtsam指定到系统`eigen`,项目主页issus好多人反馈了．还要同时指定`-DEigen3_DIR=/usr/include/`或者直接改make list

### visualization

```shell
pip install graphviz
```

```cpp
// ...
graph.saveGraph("my_test.dot", result);
```

```shell
dot -Tpng my_test.dot -o my_test.png
```


### uninstall

```shell
cd build
build$ sudo xargs rm -rf < install_manifest.txt
```

## 参考文章

- [GTSAM快速入门](https://zhuanlan.zhihu.com/p/621999120)
