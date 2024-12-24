
## constrains and concepts 约束与概念

约束(constraint)可以关联到类模板、函数模板、类模板成员函数，指定了对模板实参的一些要求，这些要求可以被用于选择最恰当的函数重载和模板特化。概念(concept) 是这些要求（即约束）的集合

### concept

A concept is a named set of requirements.

```cpp
template < 模板形参列表 >
concept 概念名 = 约束表达式;
```

```cpp
template<class T, class U>
concept isChildOf = std::is_base_of<U, T>::value;//类型约束, T必须继承自U
 
 
/***
    使用概念
    注意：概念在类型约束中接受的实参要比它的形参列表要求的要少一个，
    因为按语境推导出的类型会隐式地作第一个实参
***/
template<isChildOf<Base> T>
void f(T); // T 被 isChildOf<T, Base> 约束
```

组成概念的约束表达式也可以用requires字句定义

```cpp
#include <concepts>
 
// 概念 "Hashable" 的声明可以被符合以下条件的任意类型 T 满足：
// 对于 T 类型的值 a，表达式 std::hash<T>{}(a) 可以编译并且它的结果可以转换到 std::size_t
template<typename T>
concept Hashable = requires(T a)
{
    { std::hash<T>{}(a) } -> std::convertible_to<std::size_t>;
};
 
//在函数模板中使用概念 
template<Hashable T>
void f(T); // 受约束的 C++20 函数模板
```

也可以按以下格式使用概念

```cpp
template<typename T> requires Hashable<T> //requires子句放在template<>之后
void f(T) 
{
    //...
}
 
template<typename T>
void f(T) requires Hashable<T>  //requires子句放在函数参数列表之后
{
    //...
}
```

**概念不能递归地引用自身，也不能被约束**
Concepts cannot recursively refer to themselves and cannot be constrained:

```cpp
template<typename T>
concept V = V<T*>; // error: recursive concept
 
template<class T>
concept C1 = true;

template<C1 T>
concept Error1 = true; // Error: C1 T attempts to constrain a concept definition C1 T 尝试约束概念定义

template<class T> requires C1<T>
concept Error2 = true; // Error: the requires clause attempts to constrain a concept requires 子句尝试约束概念
```

### requires

1. requires关键字可以用来引入require子句

```cpp
template<class T>
constexpr bool is_meowable = true;
 
template<class T>
constexpr bool is_purrable() { return true; }
 
template<class T>
void f(T) requires is_meowable<T>; // OK
 
template<class T>
void g(T) requires is_purrable<T>(); // 错误：is_purrable<T>() 不是初等表达式
 
template<class T>
void h(T) requires (is_purrable<T>()); // OK
```

2. requires关键字也用来开始一个 requires 表达式

```cpp
template<typename T>
concept Addable = requires (T x) { x + x; }; // requires 表达式
 
template<typename T> requires Addable<T> // requires 子句，不是 requires 表达式
T add(T a, T b) { return a + b; }
 
template<typename T>
    requires requires (T x) { x + x; } // 随即的约束，注意关键字被使用两次
T add(T a, T b) { return a + b; }
```

require表达式具有如下语法：

```cpp
requires { 要求序列 }		
requires ( 形参列表(可选) ) { 要求序列 }
```

其中，要求序列根据复杂程度可以分为以下四种：

- 简单要求（simple requirement）
- 类型要求（type requirement）
- 复合要求（compound requirement）
- 嵌套要求（nested requirement）

```cpp
// 简单要求
template<typename T>
concept Addable = requires (T a, T b)
{
    a + b; // “表达式 a + b 是可编译的合法表达式”
};
 
// 类型要求
template<typename T>
using Ref = T&;
 
template<typename T>
concept C = requires
{
    typename T::inner; // 要求的嵌套成员名
    typename S<T>;     // 要求的类模板特化
    typename Ref<T>;   // 要求的别名模板替换
};
 
// 嵌套要求
template <class T>
concept Semiregular = DefaultConstructible<T> &&
    CopyConstructible<T> && Destructible<T> && CopyAssignable<T> &&
requires(T a, size_t n)
{  
    requires Same<T*, decltype(&a)>; // 嵌套：“Same<...> 求值为 true”
    { a.~T() } noexcept; // 复合："a.~T()" 是不抛出的合法表达式
    requires Same<T*, decltype(new T)>; // 嵌套：“Same<...> 求值为 true”
    requires Same<T*, decltype(new T[n])>; // 嵌套
    { delete new T }; // 复合
    { delete new T[n] }; // 复合
};
```

- [C++20新特性之概念、约束](https://blog.csdn.net/Jxianxu/article/details/127400217)
- [Constraints and concepts](https://en.cppreference.com/w/cpp/language/constraints)
