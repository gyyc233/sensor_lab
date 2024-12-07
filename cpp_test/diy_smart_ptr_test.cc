#include "diy_smart_ptr.hpp"
#include <memory>
#include <vector>
using namespace std;

class B; // 前置声明类B
class A {
public:
  A() { cout << "A()" << endl; }
  ~A() { cout << "~A()" << endl; }
  // shared_ptr<B> _ptr_b; // 指向B对象的智能指针
  weak_ptr<B> _ptr_b; // 指向B对象的弱智能指针。引用对象时，用弱智能指针
};
class B {
public:
  B() { cout << "B()" << endl; }
  ~B() { cout << "~B()" << endl; }
  // shared_ptr<A> _ptr_a; // 指向A对象的智能指针
  weak_ptr<A> _ptr_a; // 指向A对象的弱智能指针。引用对象时，用弱智能指针
};

int main() {
  SmartPtr<int> ptr(new int); // build on stack is ok

  SmartPtr<int> *p =
      new SmartPtr<int>(new int); // build on heap, the destructor is not called
  delete p;                       // manual call destructor

  *ptr = 20;
  std::cout << "*ptr: " << *ptr << std::endl;
  std::cout << "ptr->: " << ptr.m_ptr << std::endl;

  // pointer copy
  SmartPtr<int> ptr1(new int);
  // SmartPtr<int> ptr2(ptr1); // double free detected

  // auto_ptr
  // auto_ptr<int> p1(new int);
  // auto_ptr<int> p2 = p1;
  // *p1=10; // error

  vector<auto_ptr<int>> vec;
  vec.push_back(auto_ptr<int>(new int(10)));
  vec.push_back(auto_ptr<int>(new int(20)));
  vec.push_back(auto_ptr<int>(new int(30)));
  std::cout << "*vec[0]: " << *vec[0] << std::endl;
  // vector<auto_ptr<int>> vec2 = vec;
  // std::cout<<"*vec[0]: "<<*vec2[0]<<std::endl;

  // unique_ptr
  unique_ptr<int> u_ptr(new int);
  unique_ptr<int> u_ptr2 = std::move(u_ptr);

  // shared_ptr
  shared_ptr<A> ptr_a(new A()); // ptra指向A对象，A的引用计数为1
  shared_ptr<B> ptr_b(new B()); // ptrb指向B对象，B的引用计数为1
  // A对象的成员变量_ptrb也指向B对象，B的引用计数为1，因为是弱智能指针，引用计数没有改变
  ptr_a->_ptr_b = ptr_b;
  // B对象的成员变量_ptra也指向A对象，A的引用计数为1，因为是弱智能指针，引用计数没有改变
  ptr_b->_ptr_a = ptr_a;

  cout << ptr_a.use_count() << endl; // 打印A的引用计数结果:1
  cout << ptr_b.use_count() << endl; // 打印B的引用计数结果:1

  /**
   * 出main函数作用域，ptra和ptrb两个局部对象析构，分别给A对象和B对象的引用计数从2减到1，
   * 达不到释放A和B的条件（释放的条件是A和B的引用计数为0），因此造成两个new出来的A和B对象
   * 无法释放，导致内存泄露，这个问题就是“强智能指针的交叉引用(循环引用)问题”
   **/

  /**
   * 出main函数作用域，ptra和ptrb两个局部对象析构，分别给A对象和B对象的引用计数从1减到0，
   * 达到释放A和B的条件，因此new出来的A和B对象被析构掉，解决了“强智能指针的交叉引用(循环引用)问题”
   **/

  shared_ptr<int> aa(new int(10));
  return 0;
}
