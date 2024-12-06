#include "diy_smart_ptr.hpp"

int main(){
  SmartPtr<int> ptr(new int); // build on stack is ok

  SmartPtr<int>* p = new SmartPtr<int>(new int);  // build on heap, the destructor is not called
  delete p; // manual call destructor

  *ptr = 20;
  std::cout<<"*ptr: "<<*ptr<<std::endl;
  std::cout<<"ptr->: "<<ptr.m_ptr<<std::endl;

  // pointer copy
  SmartPtr<int> ptr1(new int);
  SmartPtr<int> ptr2(ptr1);
  return 0;
}