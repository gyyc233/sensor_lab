- [CPP module](#cpp-module)
- [syntax](#syntax)
  - [module declaration](#module-declaration)
  - [Exporting declarations and definitions 导出声明和定义](#exporting-declarations-and-definitions-导出声明和定义)
  - [Importing modules and header units](#importing-modules-and-header-units)
  - [Global module fragment 全局模块片段](#global-module-fragment-全局模块片段)
  - [Private module fragment 私有模块片段](#private-module-fragment-私有模块片段)
  - [Module partitions 模块分区](#module-partitions-模块分区)
  - [Module ownership](#module-ownership)

## CPP module

Most C++ projects use multiple translation units, and so they need to share declarations and definitions across those units. 大多数 C++ 项目使用多个翻译单元，因此它们需要在这些单元之间共享声明和定义。为此，标头的使用很突出，一个例子是标准库，其声明可以通过包含相应的标头来提供.

Modules are a language feature to share declarations and definitions across translation units. They are an alternative to some use cases of headers. 模块是一种语言功能，用于在翻译单元之间共享声明和定义。它们是某些标头用例的替代方案。

Modules are orthogonal to namespaces. 模块和命名空间是正交的

```cpp
// helloworld.cpp
export module helloworld; // module declaration
 
import <iostream>;        // import declaration
 
export void hello()       // export declaration
{
    std::cout << "Hello world!\n";
}
```

```cpp
// main.cpp
import helloworld; // import declaration
 
int main()
{
    hello();
}
```

## syntax

### module declaration

- 翻译单元可能具有模块声明，在这种情况下，它被视为模块单元
- 模块声明（如果提供）必须是翻译单元的第一个声明（全局模块片段除外）
- 每个模块单元都与模块声明中提供的模块名称相关联

```cpp
export(optional) module module-name module-partition (optional) attr (optional) ;		
```

```cpp
// (each line represents a separate独立的 translation unit)
 
export module A;   // declares the primary module interface unit for named module 'A' 声明命名模块 'A' 的主模块接口单元
module A;          // declares a module implementation unit for named module 'A' 为命名模块 'A' 声明一个模块实现单元
module A;          // declares another module implementation unit for named module 'A'
export module A.B; // declares the primary module interface unit for named module 'A.B'
module A.B;        // declares a module implementation unit for named module 'A.B'
```

### Exporting declarations and definitions 导出声明和定义

Module interface units can export declarations (including definitions), which can be imported by other translation units. To export a declaration, either prefix it with the export keyword, or else place it inside an export block. 模块接口单元可以导出声明（包括定义），这些声明可以由其他翻译单元导入。要导出声明，请在其前面加上 export 关键字，或者将其放在 export 块中。

```cpp
export declaration
export { declaration-seq(optional) }
```

```cpp
export module A; // declares the primary module interface unit for named module 'A'
 
// hello() will be visible by translations units importing 'A'
export char const* hello() { return "hello"; } 
 
// world() will NOT be visible.
char const* world() { return "world"; }
 
// Both one() and zero() will be visible.
export
{
    int one()  { return 1; }
    int zero() { return 0; }
}

// Exporting namespaces also works: hi::english() and hi::french() will be visible.
export namespace hi
{
    char const* english() { return "Hi!"; }
    char const* french()  { return "Salut!"; }
}
```

### Importing modules and header units

```cpp
/////// A.cpp (primary module interface unit of 'A')
export module A;
 
export char const* hello() { return "hello"; }
 
/////// B.cpp (primary module interface unit of 'B')
export module B;
 
export import A;
 
export char const* world() { return "world"; }
 
/////// main.cpp (not a module unit)
#include <iostream>
import B;
 
int main()
{
    std::cout << hello() << ' ' << world() << '\n';
}
```

- #include 不应在模块单元中使用，相反，也可以使用 import 声明将头文件导入：


```cpp
// A.cpp (primary module interface unit of 'A')
export module A;
 
import <iostream>;
export import <string_view>;
 
export void print(std::string_view message)
{
    std::cout << message << std::endl;
}
 
/////// main.cpp (not a module unit)
import A;
 
int main()
{
    std::string_view message = "Hello, world!";
    print(message);
}
```

### Global module fragment 全局模块片段

- 模块单元可以以全局模块片段为前缀

```cpp
// A.cpp (primary module interface unit of 'A')
module;
 
// Defining _POSIX_C_SOURCE adds functions to standard headers,
// according to the POSIX standard.
#define _POSIX_C_SOURCE 200809L
#include <stdlib.h>
 
export module A;
 
import <ctime>;
 
// Only for demonstration (bad source of randomness).
// Use C++ <random> instead.
export double weak_random()
{
    std::timespec ts;
    std::timespec_get(&ts, TIME_UTC); // from <ctime>
 
    // Provided in <stdlib.h> according to the POSIX standard.
    srand48(ts.tv_nsec);
 
    // drand48() returns a random number between 0 and 1.
    return drand48();
}
 
/////// main.cpp (not a module unit)
import <iostream>;
import A;
 
int main()
{
    std::cout << "Random value between 0 and 1: " << weak_random() << '\n';
}
```

### Private module fragment 私有模块片段

主模块接口单元可以以私有模块片段为后缀，这允许将模块表示为单个翻译单元，而无需使导入器可以访问模块的所有内容。

```cpp
export module foo;
 
export int f();
 
module : private; // ends the portion of the module interface unit that
                  // can affect the behavior of other translation units
                  // starts a private module fragment
 
int f()           // definition not reachable from importers of foo
{
    return 42;
}
```

### Module partitions 模块分区

一个模块可以有模块分区单元。它们是模块单元，其模块声明包括一个模块分区，该分区以冒号 ： 开头，并放在模块名称之后

```cpp
export module A:B; // Declares a module interface unit for module 'A', partition ':B'.
```

usage

```cpp
///////  A.cpp   
export module A;     // primary module interface unit
 
export import :B;    // Hello() is visible when importing 'A'.
import :C;           // WorldImpl() is now visible only for 'A.cpp'.
// export import :C; // ERROR: Cannot export a module implementation unit.
 
// World() is visible by any translation unit importing 'A'.
export char const* World()
{
    return WorldImpl();
}
```

```cpp
/////// A-B.cpp 
export module A:B; // partition module interface unit
 
// Hello() is visible by any translation unit importing 'A'.
export char const* Hello() { return "Hello"; }
```

```cpp
/////// A-C.cpp 
module A:C; // partition module implementation unit
 
// WorldImpl() is visible by any module unit of 'A' importing ':C'.
char const* WorldImpl() { return "World"; }
```

```cpp
/////// main.cpp 
import A;
import <iostream>;
 
int main()
{
    std::cout << Hello() << ' ' << World() << '\n';
    // WorldImpl(); // ERROR: WorldImpl() is not visible.
}
```

### Module ownership

```cpp
export module lib_A;
 
int f() { return 0; } // f has module linkage
export int x = f();   // x equals 0
```

```cpp
export module lib_B;
 
int f() { return 1; } // OK, f in lib_A and f in lib_B refer to different entities
export int y = f(); // y equals 1
```

```cpp
/////// decls.h
int f(); // #1, attached to the global module
int g(); // #2, attached to the global module
```

```cpp
/////// Module interface of M
module;
#include "decls.h"
export module M;
export using ::f; // OK, does not declare an entity, exports #1
int g();          // Error: matches #2, but attached to M
export int h();   // #3
export int k();   // #4
```

```cpp
/////// Other translation unit
import M;
static int h();   // Error: matches #3
int k();          // Error: matches #4
```

以下声明未附加到任何命名模块（因此可以在模块外部定义声明的实体）：

- 具有外部链接的命名空间定义;
- 语言链接规范中的声明

```cpp
export module lib_A;
 
namespace ns // ns is not attached to lib_A.
{
    export extern "C++" int f(); // f is not attached to lib_A.
           extern "C++" int g(); // g is not attached to lib_A.
    export              int h(); // h is attached to lib_A.
}
// ns::h must be defined in lib_A, but ns::f and ns::g can be defined elsewhere (e.g.
// in a traditional source file).
```
