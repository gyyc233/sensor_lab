- [新关键字](#新关键字)
- [改进的关键字用法](#改进的关键字用法)
- [三路比较运算符(\<=\>)](#三路比较运算符)
- [概念](#概念)
- [协程](#协程)
- [模块](#模块)
- [范围](#范围)
- [lambda改进](#lambda改进)
- [using 拓展](#using-拓展)
- [immediate functions 即时函数](#immediate-functions-即时函数)
- [char8\_t 类型](#char8_t-类型)
- [标准库特性](#标准库特性)


# 新关键字

1. co_await: 关键字用于暂停当前协程，直到被 awaitable 对象的操作完成。

```cpp
std::future<int> async_computation() {
    co_await std::suspend_never{};
    co_return 42;
}
```

2. co_return: 用于在协程中返回一个值，并终止协程

```cpp
std::future<int> async_computation() {
    co_return 42;
}
```

3. co_yield: 可以用于编写生成器，这种生成器可以在需要时一次生成一个值，并通过 co_yield 将该值返回给调用者

```cpp
Generator<int> sequence() {
    for (int i = 0; i < 10; ++i) {
        co_yield i;
    }
}
```

4. concept: 用于定义概念，它是一组对类型进行约束的条件

```cpp
template<typename T>
concept Incrementable = requires(T t) {
    ++t;
    t++;
};
```

约束表达式是定义概念的核心，其形式为带有 requires 关键字的布尔表达式

5. requires: 用于定义约束表达式（constraints expressions）。requires 表达式允许我们在模板定义中指定特定的约束条件，从而限制模板参数的类型。该表达式在某些条件满足时为true，否则为false

```cpp
template<typename T>
requires std::integral<T>
T add(T a, T b) {
    return a + b;
}
```

6. consteval: 用于声明必须在编译时求值的常量表达式函数。consteval 函数只能被编译时常量上下文调用，如果在运行时调用这些函数，编译器将会报错

```cpp
consteval int square(int n) {
    return n * n;
}
```

7. constinit: 用于声明变量在运行时具有静态存储持续时间且必须被立即初始化。这可以防止未初始化的静态变量带来的问题，同时防止变量没有在编译时就初始化的情况，确保了变量的初始化顺序

```cpp
constinit int value = 10;
```

# 改进的关键字用法

1. constexpr

- constexpr 虚函数：虚函数可以是 constexpr。
- constexpr 动态分配：在 constexpr 上下文中支持动态内存分配。
- constexpr try-catch：在 constexpr 函数中支持 try-catch 异常处理。
- constexpr 构造函数：构造函数可以是 constexpr，甚至支持更加复杂的初始化逻辑。

constexpr 动态分配

```cpp
#include <iostream>
#include <memory>
​
constexpr int* createArray(int n) {
    int* arr = new int[n];
    for (int i = 0; i < n; ++i) {
        arr[i] = i * i;
    }
    return arr;
}
​
int main() {
    constexpr int size = 5;
    constexpr int* arr = createArray(size);
​
    for (int i = 0; i < size; ++i) {
        std::cout << arr[i] << std::endl; // 输出各个元素的平方
    }
​
    delete[] arr; // 清理动态分配的内存
    return 0;
}
```

constexpr try-catch

```cpp
#include <iostream>

constexpr int divide(int a, int b) {
    if (b == 0) {
        throw "Division by zero!";
    }
    return a / b;
}

int main() {
    try {
        constexpr int result = divide(10, 2);
        std::cout << "Result: " << result << std::endl;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
    }

    return 0;
}
```

constexpr 构造函数

```cpp
#include <iostream>

class Point {
public:
    constexpr Point(int x, int y) : x_(x), y_(y) {}

    constexpr int getX() const { return x_; }
    constexpr int getY() const { return y_; }

private:
    int x_;
    int y_;
};

int main() {
    constexpr Point p(3, 4);
    constexpr int x = p.getX();
    constexpr int y = p.getY();
    std::cout << "Point: (" << x << ", " << y << ")" << std::endl; // Point: (3, 4)

    return 0;
}
```

2. inline: inline 变量允许在多个翻译单元（translation units）中定义同一变量，而不会导致链接错误。这在全局变量和静态成员变量中尤其有用

```cpp
// header.h
#ifndef HEADER_H
#define HEADER_H

inline int count = 0; // 允许在多个翻译单元中定义

#endif // HEADER_H

// main.cpp
#include "header.h"
#include <iostream>

void increment() {
 ++count;
}

int main() {
 increment();
 std::cout << "Count: " << count << std::endl; // Count: 1
 return 0;
}

// other.cpp
#include "header.h"

void reset() {
 count = 0;
}
```

3. decltype(auto): 用于函数返回类型，推断返回值的精确类型

```cpp
auto create_array() -> decltype(auto) {
    return std::array<int, 3>{1, 2, 3};
}
```

# 三路比较运算符(<=>)

`左操作数 <=> 右操作数`

三路运算符返回的是一个对象，如下所示：

- 如果左操作数 < 右操作数 则 (a <=> b) < 0
- 如果左操作数 > 右操作数 则 (a <=> b) > 0
- 如果左操作数 和 右操作数 相等/等价则 (a <=> b) == 0
- 三路运算符的返回结果和我们实际中使用的strcmp、strncmp一样，但是又有本质的区别，三路运算符返回的是std::strong_ordering、std::weak_ordering以及std::partial_ordering对象，而strcmp和strncmp返回的是整型数据。

```cpp
class Point {
  int x;
  int y;
public:
  Point(int _x,int _y):x(_x),y(_y){};
  //预制生成默认比较函数，编译器会生成<,==,<,<=,>=,!=运算代码
  auto operator<=>(const Point&) const = default;
};
int main() {
  Point pt1{1, 1}, pt2{1, 2};
  std::set<Point> s; // ok
  s.insert(pt1);     // ok
  std::cout << std::boolalpha
    << (pt1 == pt2) << ' ' // false；operator== 隐式地采用默认版本
    << (pt1 != pt2) << ' ' // true
    << (pt1 <  pt2) << ' ' // true
    << (pt1 <= pt2) << ' ' // true
    << (pt1 >  pt2) << ' ' // false
    << (pt1 >= pt2) << ' ';// false
}
```

# 概念

# 协程

1. std::coroutine_handle: 这是一个泛化的句柄类型,用于操作协程
2. std::suspend_always: 这是一个可悬挂对象，总是悬挂协程
3. std::suspend_never: 这是一个可悬挂对象，从不悬挂协程

# 模块

# 范围

# lambda改进

1. Lambda 默认构造函数: Lambda 可以在某些情况下被默认构造
2. 模板 Lambda:  允许 Lambda 表达式直接使用模板参数
3. 捕获包扩展: 可以捕获多参数包到 Lambda 捕获列表
4. constexpr Lambda: 使其表达式能在编译时计算
5. 使用 template 关键字的 Lambda: 允许使用 template 关键字定义模板参数
6. 捕获 this 的移动对象: [obj = std::move(*this)] 的方式来捕获当前对象，从而实现移动捕获
7. Lambda 表达式支持捕获外部对象的一部分，并且能通过编译时明确捕获其值或引用

# using 拓展

# immediate functions 即时函数

通过使用关键字 consteval 将其定义为显式常量求值函数。即时函数在编译期进行求值，与 constexpr 相比，consteval 的函数必须在编译期求值，若在运行期调用则会导致编译错误

关键特性
- 编译期求值：即时函数只能在编译期进行求值。
- 严格常量求值：即时函数必须在编译期被求值，不能在运行期调用。
- 显式约束：使用 consteval 关键字显式声明，使得开发者明确知道该函数必须在编译期求值。

# char8_t 类型

用来处理 UTF-8 编码的字符类型。之前，UTF-8 编码通常使用 char 类型来表示

```cpp
char8_t utf8_char = u8'a'; // 单个 UTF-8 编码字符
u8"Hello, UTF-8!"          // UTF-8 编码的字符串字面量
```

# 标准库特性

1. 格式化库 std::format

与传统的 printf 和 std::stringstream 方法相比，std::format 更加简洁且安全，支持类型安全的格式化

基本用法

```cpp
#include <iostream>
#include <format>

int main() {
    int num = 42;
    std::string formatted_str = std::format("The answer is: {}", num);
    std::cout << formatted_str << std::endl; // 输出: The answer is: 42
    return 0;
}
```

基本占位符{}

```cpp
    std::cout << std::format("Name: {}, Age: {}\n", name, age);
    std::cout << std::format("Pi: {:.2f}\n", pi); // 输出到小数点后两位
```

指定宽度与填充字符

```cpp
    int value = 42;
    std::cout << std::format("{:>10}\n", value); // 右对齐，宽度为10
    std::cout << std::format("{:<10}\n", value); // 左对齐，宽度为10
    std::cout << std::format("{:0>10}\n", value); // 用0填充，宽度为10
```

使用不同的进制表示数值

```cpp
    std::cout << std::format("Decimal: {}\n", number);
    std::cout << std::format("Hexadecimal: {:#x}\n", number); // 带有前缀0x
    std::cout << std::format("Octal: {:#o}\n", number);       // 带有前缀0
    std::cout << std::format("Binary: {:#b}\n", number);      // 带有前缀0b
```

用户自定义类型的格式化

通过特化 std::formatter 模板，可以为用户自定义类型提供格式化支持

```cpp
#include <iostream>
#include <format>

struct Point {
    int x, y;
};

template <>
struct std::formatter<Point> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.end(); }

    auto format(const Point& p, std::format_context& ctx) {
        return std::format_to(ctx.out(), "({}, {})", p.x, p.y);
    }
};

int main() {
    Point pt{3, 4};

    std::cout << std::format("Point: {}\n", pt);
    return 0;
}
```

2. std::span

在不复制数据的情况下提供对数组或容器的一段连续元素的访问。它类似于指针，但比指针更安全、更便捷。std::span 提供了一种轻量级的方式来引用现有的数据，不拥有数据，也不管理其生命周期

3. std::jthread

std::jthread 自动处理线程的加入（join）操作，避免了程序员忘记加入线程导致的资源泄露问题。此外，std::jthread 还内置了对停止令牌（stop token）的支持，使得线程的停止操作更加简单和安全

4. 改进的智能指针

- 增加了对std::shared_ptr原子操作的支持，这意味着可以安全地在多线程环境中对std::shared_ptr对象进行读写操作，而不需要额外的同步机制。这些原子操作包括std::atomic_load、std::atomic_store、std::atomic_exchange
- 可以直接使用std::weak_ptr来创建std::shared_ptr，而不需要先调用std::weak_ptr::lock。这使得代码更简洁，并且在某些情况下可以提高性能
- std::enable_shared_from_this获得了一个新的成员函数weak_from_this，它返回一个std::weak_ptr，指向enable_shared_from_this对象的当前实例。这使得从当前对象获取std::weak_ptr变得更加容易
- std::dynamic_pointer_cast 的优化

[参考文章](https://zhuanlan.zhihu.com/p/703576234)
