#include <algorithm>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

int fun(int a, int b, int c, int d, int e) { return a + b - c + d - e; }

class Test {
public:
  int func(int a, int b, int c, int d, int e) { return a + b - c + d - e; }
  static int s_func(int a, int b, int c, int d, int e) {
    return a + b - c + d - e;
  }
};

void hello() { std::cout << "hello world!" << std::endl; }
void hello_str(std::string str) { std::cout << str << std::endl; }

template <typename T> T sum(T a, T b) { return a + b; }

// 函数对象
class PrintAdd1 {
public:
  void operator()(int left, int right) {
    std::cout << "sum : " << left + right << std::endl;
  }
};

// 模板函数对象
template <typename T> class PrintAdd2 {
public:
  void operator()(T left, T right) {
    std::cout << "sum : " << left + right << std::endl;
  }
};

// 类静态成员函数
class StaticClass1 {
public:
  static void hello_static(std::string s) { std::cout << s << std::endl; }
};

// 模板类静态成员函数
template <typename T> class StaticClass2 {
public:
  static void hello_static(T out) { std::cout << out << std::endl; }
};

// 普通类成员函数
class B_Test {
public:
  void hello(std::string str) { std::cout << str << std::endl; }
};

// 模板类成员函数
template <typename T> class B_Test2 {
public:
  void hello(T str) { std::cout << str << std::endl; }
};

int main() {
  // bind 绑定普通函数与函数指针
  int x = 1, y = 2, z = 3;
  auto g =
      std::bind(&fun, x, y, std::placeholders::_1, z,
                std::placeholders::_2); //  //第一个参数&可省略 但最好写成&fun
  std::cout << g(11, 22) << std::endl; // // fun(1, 2, 22, 3, 11) => 1+2-22+3-11
  std::cout << std::bind(fun, x, y, std::placeholders::_2, z,
                         std::placeholders::_1)(11, 22)
            << std::endl; //等价

  auto f = std::bind(&Test::func, Test(), x, y, std::placeholders::_1,
                     std::placeholders::_3,
                     std::placeholders::_2); // Test() 作用像是this指针
  std::cout << f(10, 6, 7) << std::endl;     // 1+2-10+7-6 = -6
  std::cout << f.operator()(10, 6, 7) << std::endl;

  auto h = std::bind(&Test::s_func, x, y, std::placeholders::_1,
                     std::placeholders::_3, std::placeholders::_2);
  std::cout << h(10, 6, 7) << std::endl; // 1+2-10+7-6 = -6
  std::cout << h.operator()(10, 6, 7) << std::endl;

  // std::function
  // 普通函数
  std::function<void()> func1 =
      &hello; // or std::function<void()> func1(&hello)
  func1();    // call func1.operator()() ---> void hello()
  std::function<void(std::string)> func2 = &hello_str;
  func2("hello world!"); // call func2.operator()(string) ---> void
                         // hello_str(string)

  // 模板函数
  std::function<int(int, int)> func3 =
      sum<int>; // call func3.operator()(int, int) ---> int sum(int, int)
  std::cout << func3(3, 5) << std::endl;

  // lambda 表达式
  std::function<int(int, int)> func4 = [](int a, int b) -> int {
    std::cout << "lambda" << std::endl;
    return a + b;
  };
  std::cout << func4(3, 5) << std::endl; //调用func1.operator()(int, int) ==>
                                         //调用lambda表达式返回求和结果

  // 函数对象
  std::function<void(int, int)> func_5 =
      PrintAdd1(); // 调用默认无参构造函数创建匿名类对象给func_5
  func_5(3, 5);

  // 模板函数对象
  std::function<void(int, int)> func_6 = PrintAdd2<int>();
  func_5(11, 99);

  // 类静态成员函数
  std::function<void(std::string)> func_7 = &StaticClass1::hello_static;
  // call void hello_static(string)
  func_7("static hello world");

  // 模板类静态成员函数
  std::function<void(std::string)> func_8 =
      &StaticClass2<std::string>::hello_static;
  // call void StaticClass2<std::string>::hello_static(string)
  func_8("template static hello world");

  // 普通成员变量
  B_Test b_test;
  std::function<void(B_Test *, std::string)> func_9 = &B_Test::hello;
  func_9(&b_test, "call B_Test::hello");

  // 模板类成员函数
  B_Test2<std::string> b_test2;
  std::function<void(B_Test2<std::string> *, std::string)> func_10 =
      &B_Test2<std::string>::hello;
  func_10(&b_test2, "call B_Test2::hello");

  return 0;
}
