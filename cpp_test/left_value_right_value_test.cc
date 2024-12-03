#include <cstdint>
#include <cstring>
#include <iostream>

class TestClassA {
public:
  TestClassA(int size) : m_size(size) {
    m_data = new char[m_size];
    std::cout << "ordinary construction size: " << size << std::endl;
  }

  TestClassA(const TestClassA &A) : m_size(A.m_size) {
    m_data = new char[m_size];
    memcpy(m_data, A.m_data, m_size);
    std::cout << "deep copy, size: " << m_size << std::endl;
  }

  TestClassA(TestClassA &&A) : m_size(A.m_size), m_data(A.m_data) {
    std::cout << "remove reference " << m_size << std::endl;
    A.m_data = nullptr;
  }

  ~TestClassA() {
    if (m_data) {
      delete m_data;
      m_data = nullptr;
      std::cout << "destroy size: " << m_size << std::endl;
    }
  }

  int m_size;
  char *m_data;
};

void TestLValueOrRValue(TestClassA &rA) {
  std::cout << "left Value " << rA.m_size << std::endl;
}

void TestLValueOrRValue(TestClassA &&rA) {
  std::cout << "right Value " << rA.m_size << std::endl;
}

void TestFun() {
  TestLValueOrRValue(TestClassA(1111));
  TestClassA &&ra2222 = TestClassA(2222);
  TestLValueOrRValue(ra2222);
}

int main() {
  TestClassA a1(999);
  TestClassA a2(std::move(a1));

  TestClassA a3(TestClassA(1000));

  TestFun();
}
