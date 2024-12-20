
## CPP14 some features

1. 函数返回值类型推导

-  非虚函数支持使用 auto 进行返回值类型推导
-  如果函数是虚函数,则不能使用返回值类型推导。虚函数的返回类型必须显式指定，用于设置虚函数表，进行动态绑定

2. 泛型 lambda 表达式

- 允许在 lambda 表达式中使用 auto 参数类型

```cpp
auto lambda = [](auto x, auto y) {
    return x + y;
};
```

3. 变量模板

```cpp
template<class T>
constexpr T pi = T(3.1415926535897932385L);

int main() {
    cout << pi<int> << endl; // 3
    cout << pi<double> << endl; // 3.14159
    return 0;
}
```

4. 别名模板

```cpp
template<typename T, typename U>
struct A {
    T t;
    U u;
};

template<typename T>
using B = A<T, int>;

int main() {
    B<double> b;
    b.t = 10;
    b.u = 20;
    cout << b.t << endl;
    cout << b.u << endl;
    return 0;
}
```

5. std::make_unique

提供了 std::make_unique 函数，用于创建 std::unique_ptr，类似于 std::make_shared

```cpp
struct A {};
std::unique_ptr<A> ptr = std::make_unique<A>();
```

6. 返回值类型后置

```cpp
auto add(int a, int b) -> int {
    return a + b;
}
```

7. 初始化列表的改进

支持在更多地方使用初始化列表，例如在 if 和 switch 语句

```cpp
if (std::initializer_list<int> list = {1, 2, 3}; !list.empty()) {
    // do something
}
```

8. [[deprecated]]标记

增加了deprecated标记，修饰类、变、函数等，当程序中使用到了被其修饰的代码时，编译时被产生警告，用户提示开发者该标记修饰的内容将来可能会被丢弃，尽量不要使用

```cpp
struct [[deprecated]] A { };

int main() {
    A a;
    return 0;
}
```
