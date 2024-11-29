
// 可变参数模板
// 常见定义
// template <typename... T>　// Ｔ是模板参数包,任意数目和任意类型
// void f(T... args);　//args是函数参数包

// 1. 声明一个参数包T... args，这个参数包中可以包含0到任意个模板参数,
// 把带省略号的参数称为　参数包，
// 无法直接获取参数包args中的每个参数的，只能通过「展开参数包」的方式来获取参数包中的每个参数

// 2. 在模板定义的右边，可以将参数包展开成一个一个独立的参数

// 3. ...在参数右侧，代表一组实参， ...在参数左侧表示可变参数

// 转载链接 https://www.cnblogs.com/S1mpleBug/p/16834298.html

// 展开可变模板参数函数的方法：1. 通过[递归函数]展开参数包;2.
// 通过[逗号表达式]展开参数包

#include <functional>
#include <iostream>
#include <string>
#include <vector>

template <typename... T> void f(T... args) {
  std::cout << sizeof...(args) << std::endl;
}

/// @brief recursive end function
/// @note 当没有参数时，调用该函数终止递归
void print() { std::cout << "empty" << std::endl; }

// @note expand args
template <typename T, typename... Args> void print(T head, Args... rest) {
  std::cout << "parameter " << head << std::endl;
  print(rest...);
}

template <typename T> T sum(T t) { return t; }

template <typename T, typename... Types> T sum(T first, Types... rest) {
  return first + sum<T>(rest...);
}

template <typename T> void printArg(T t) { std::cout << t << std::endl; }

template <typename... Args> void expand(Args... args) {
  std::vector<int> arr = {(printArg(args), 0)...}; // 逗号表达式
  // 这个数组最终会被展开成 {((printArg(arg1), 0), (printArg(arg2), 0),
  // (printArg(arg3), 0),  etc...)}
  // 这个数组的目的纯粹是为了在数组构造的过程展开参数包,借助了[列表初始化]

  // 介绍逗号表达式
  // d=(a=b,c)
  // 这个表达式会按顺序执行：b会先赋值给a，接着括号中的逗号表达式返回c的值，因此d将等于c

  for (auto &it : arr) {
    std::cout << it << ", ";
  }
  std::cout << std::endl;
}

// 上诉例子，将函数作为参数，使用lambda
template <typename T, typename... Args>
void expand_lambda(const T &func, Args &&... args) {
  // 这里用到了完美转发
  int arr[] = {(func(std::forward<Args>(args)), 0)...};
  // initializer_list<int>{ (func(std::forward<Args>(args)), 0)... };

  // 这里的Ｔ类型是 function<void int>, 类型是`const function<void(int)> &func`
  // or `function<void(int)> func`
}

// 可变参数模板类的展开一般需要定义两到三个类，包括「类声明」和「偏特化」的模板类。如下方式定义了一个基本的可变参数模板类：
// 这个Sum类的作用是在编译期计算出参数包中参数类型的size之和，通过Sum<int,
// double, short>::value就可以获取这3个类型的size之和为14。

// 前向声明, 声明Sum是一个可变参数模板类
template <typename... Args> struct Sum;

// 基本定义
// 它定义了一个部分展开的可变参数模板类，告诉编译器如何递归展开参数包
// 这里要求Sum的模板参数至少有一个，因为可变参数模板中的模板参数可以有0个，有时候0个模板参数没有意义，就可以通过上面的声明方式来限定模板参数不能为0个
template <typename First, typename... Rest> struct Sum<First, Rest...> {
  enum { value = Sum<First>::value + Sum<Rest...>::value };
};

// 递归终止
template <typename Last> struct Sum<Last> {
  enum { value = sizeof(Last) };
};

int main() {
  // simple
  f();
  f(1, 2);
  f(1, 2, "");

  // 使用递归函数展开参数包
  print(1, 2, 3, 4);

  // 使用可变参数模板求和
  std::cout << sum(1, 2, 3, 4) << std::endl;
  // 递归函数展开参数包是一种标准做法，也比较好理解，但也有一个缺点,就是「必须」要一个重载的递归终止函数

  // 借助「逗号表达式」和「初始化列表」展开参数包
  expand(1, 2, 3, 4);

  // expand_lambda([](int i)->void{std::cout<<i<<std::endl;},1,2,3);
  expand_lambda([](auto i) -> void { std::cout << i << std::endl; }, 1, 2,
                "hello"); // 泛化lambda C++14 feature

  // 可变参数模板类
  // std::tuple就是一个可变模板类，它的定义如下：
  // template<typename... Types>
  // class tuple;

  std::tuple<> tp0;
  std::tuple<int> tp1 = std::make_tuple(1);
  std::tuple<int, double> tp2 = std::make_tuple(1, 2.5);
  std::tuple<int, double, std::string> tp3 = std::make_tuple(1, 2.5, "");

  // 可变参数模板类的参数包展开需要通过「模板特化」和「继承方式」去展开，展开方式比可变参数模板函数要复杂
  std::cout << Sum<int, double, short>::value << std::endl;

  return 0;
}
