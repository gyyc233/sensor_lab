- [build sophus via source](#build-sophus-via-source)
- [build error](#build-error)
- [set CMakeLists.txt](#set-cmakeliststxt)

## build sophus via source

```bash
cd ~/src
git clone https://github.com/strasdat/Sophus.git

cd ~/src/Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
make
```

## build error

```bash
Sophus/sophus/so2.cpp:32:26: error: lvalue required as left operand of assignment
   32 |   unit_complex_.real() = 1.;
      |                          ^~
/home/V01/uids0025/src/Sophus/sophus/so2.cpp:33:26: error: lvalue required as left operand of assignment
   33 |   unit_complex_.imag() = 0.;

```

solution:

modified so2.cpp:

```cpp
// error
SO2::SO2()
{
unit_complex_.real() = 1.;
unit_complex_.imag() = 0.;
}

// fixed
SO2::SO2()
{
//unit_complex_.real() = 1.;
//unit_complex_.imag() = 0.;
unit_complex_.real(1.);
unit_complex_.imag(0.);
}
```

## set CMakeLists.txt

```bash
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# updated
# Sophus 
set(Sophus_DIR /home/my_name/src/Sophus/build)#路径改为自己的安装路径
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
```
