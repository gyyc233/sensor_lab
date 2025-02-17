- [CMakeCache 缓存](#cmakecache-缓存)
- [build clean](#build-clean)

## CMakeCache 缓存

使用cmake进行编译时，经常因为要新增一个execute, 一个新.so .a 此时cmake没有get到，导致这些新的包没有参与编译，我经常傻傻的把build删掉再全部重新编．现在发现`CMakeCache.txt`这个cmake 缓存(太好了，是`CMakeCache.txt`，风扇有救了)我不用再等那么久了

- 清除缓存，不想从头重新编译，可以只删除 build/CMakeCache.txt 再重新`cmake ..`
- 这文件里面装的就是缓存的变量，删了他就可以让 CMake 强制重新检测一遍所有库和编译器
- `CMakeCache.txt`保存了所有的编译选项，别忘记你的optional

## build clean

当源文件发生改变时，只需要`make clean`重新编译
