#include <iostream>

template <typename T> class SmartPtr {
public:
  SmartPtr(T *ptr = nullptr) : m_ptr(ptr) {
    // 这里是浅拷贝
    std::cout << "SmartPtr construction" << std::endl;
  }
  ~SmartPtr() {
    std::cout << "SmartPtr destruction" << std::endl;
    delete m_ptr;
  }

  // rewrite * and -> operator

  // 取地址
  T &operator*() { return *m_ptr; }

  T *operator->() { return m_ptr; }

public:
  T *m_ptr;
};
