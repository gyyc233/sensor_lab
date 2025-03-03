- [lambda 函数结构](#lambda-函数结构)
  - [Lambda 在编译期的实现](#lambda-在编译期的实现)
  - [`[=] [&] [this] []`对this指针的捕获情况](#--this-对this指针的捕获情况)
  - [为什么引入\[\*this\]](#为什么引入this)

# lambda 函数结构

```cpp
[capture_list](parameter_list) -> return_type {function_body}
```

定义一个lambda

```cpp
#include <iostream>
class Foo {
public:
    void show() {
       auto print = [=](int a){
            std::cout << m_x << a <<std::endl;
        };
        int x = 233;
        print(x);
    }

private:
    int m_x;
};

int main() {
	Foo foo;
  	foo.show();
}
```

这里选择了 by-copy (=) 的方法来捕获 x 这个变量，也就是会复制一份 x 进到 print lambda 里，那么这个值 copy 到了哪里呢

## Lambda 在编译期的实现

使用[cppinsights](https://cppinsights.io/)查看可能的实现

```cpp
// C++11
#include <iostream>
class Foo
{
  
  public: 
  inline void show()
  {
        
    class __lambda_5_21
    {
      public: 
      inline void operator()(int a) const
      {
        std::cout.operator<<(__this->m_x).operator<<(a).operator<<(std::endl);
      }
      
      private: 
      Foo * __this;
      public: 
      // inline /*constexpr */ __lambda_5_21(__lambda_5_21 &&) noexcept = default;
      __lambda_5_21(const Foo * _this): __this{_this}{}
    };
    
    __lambda_5_21 print = __lambda_5_21(__lambda_5_21{this});
    int x = 233;
    print.operator()(x);
  }
  
  
  private: 
  int m_x;
  public: 
  // inline Foo() noexcept = default;
};


int main()
{
  Foo foo;
  foo.show();
  return 0;
}
```

- 在show函数中生成一个唯一类`__lambda_5_21`, 有一个`__lambda_5_21`实例`print`，用this构造`print`
- 重写仿函数`operator()` 带const

改为引用传递后的cppinsights显示的结果没有变化

```cpp
#include <iostream>
class Foo
{
  
  public: 
  inline void show()
  {
        
    class __lambda_5_21
    {
      public: 
      inline void operator()(int a) const
      {
        std::cout.operator<<(__this->m_x).operator<<(a).operator<<(std::endl);
      }
      
      private: 
      Foo * __this;
      public: 
      // inline /*constexpr */ __lambda_5_21(__lambda_5_21 &&) noexcept = default;
      __lambda_5_21(const Foo * _this)
      : __this{_this}
      {}
      
    };
    
    __lambda_5_21 print = __lambda_5_21(__lambda_5_21{this});
    int x = 233;
    print.operator()(x);
  }
  
  
  private: 
  int m_x;
  public: 
  // inline Foo() noexcept = default;
};


int main()
{
  Foo foo;
  foo.show();
  return 0;
}
```

使用`[this]`捕获,也没有变化

```cpp
#include <iostream>
class Foo
{
  
  public: 
  inline void show()
  {
        
    class __lambda_5_21
    {
      public: 
      inline void operator()(int a) const
      {
        std::cout.operator<<(__this->m_x).operator<<(a).operator<<(std::endl);
      }
      
      private: 
      Foo * __this;
      public: 
      // inline /*constexpr */ __lambda_5_21(__lambda_5_21 &&) noexcept = default;
      __lambda_5_21(const Foo * _this)
      : __this{_this}
      {}
      
    };
    
    __lambda_5_21 print = __lambda_5_21(__lambda_5_21{this});
    int x = 233;
    print.operator()(x);
  }
  
  
  private: 
  int m_x;
  public: 
  // inline Foo() noexcept = default;
};


int main()
{
  Foo foo;
  foo.show();
  return 0;
}
```

改为`[]`则不会捕获`this`

```cpp
#include <iostream>
class Foo
{
  
  public: 
  inline void show()
  {
        
    class __lambda_5_21
    {
      public: 
      inline void operator()(int a) const
      {
        std::cout.operator<<(a).operator<<(std::endl);
      }
      
      using retType_5_21 = void (*)(int);
      inline operator retType_5_21 () const noexcept
      {
        return __invoke;
      };
      
      private: 
      static inline void __invoke(int a)
      {
        __lambda_5_21{}.operator()(a);
      }
      
      public: 
      // inline /*constexpr */ __lambda_5_21(__lambda_5_21 &&) noexcept = default;
      
    };
    
    __lambda_5_21 print = __lambda_5_21(__lambda_5_21{});
    int x = 233;
    print.operator()(x);
  }
  
  
  private: 
  int m_x;
  public: 
  // inline Foo() noexcept = default;
};


int main()
{
  Foo foo;
  foo.show();
  return 0;
}
```

## `[=] [&] [this] []`对this指针的捕获情况

- `[=] [&] [this]` 对this指针为引用捕获, `[]`不捕获this指针
- C++17新增`[*this]`对`this`指针进行值捕获
- lambda仿函数均为const,不改变成员变量

[*this]使用

```cpp
#include <iostream>
class Foo {
public:
    void show() const {
       auto print = [*this](int a) {
            std::cout << m_x << a <<std::endl;
        };
		int x = 233;
		print(x);
    }

private:
    int m_x;
};

int main() {
	Foo foo;
  	foo.show();
}
////////////////////////////////
  inline void show() const
  {
        
    class __lambda_5_21
    {
      public: 
      inline /*constexpr */ void operator()(int a) const
      {
        std::cout.operator<<((&__this)->m_x).operator<<(a).operator<<(std::endl);
      }
      
      private: 
      const Foo __this;
      public: 
      // inline /*constexpr */ __lambda_5_21 & operator=(const __lambda_5_21 &) /* noexcept */ = delete;
      __lambda_5_21(const Foo & _this)
      : __this{_this}
      {}
      
    };
    
    __lambda_5_21 print = __lambda_5_21{*this};
    int x = 233;
    print.operator()(x);
  }
```

那么使用`mutable`之后，没有了 const，也可以正常修改 this 的属性 m_x

```cpp
#include <iostream>
class Foo {
public:
    void show() {
       auto print = [&](int a)mutable {
         	m_x++;
            std::cout << m_x << a <<std::endl;
        };
		int x = 233;
		print(x);
    }

private:
    int m_x;
};

int main() {
	Foo foo;
  	foo.show();
}

////////////////////////

class Foo
{
  
  public: 
  inline void show()
  {
        
    class __lambda_5_21
    {
      public: 
      inline void operator()(int a)
      {
        __this->m_x++;
        std::cout.operator<<(__this->m_x).operator<<(a).operator<<(std::endl);
      }
      
      private: 
      Foo * __this;
      public: 
      // inline /*constexpr */ __lambda_5_21(__lambda_5_21 &&) noexcept = default;
      __lambda_5_21(Foo * _this)
      : __this{_this}
      {}
      
    };
    
    __lambda_5_21 print = __lambda_5_21(__lambda_5_21{this});
    int x = 233;
    print.operator()(x);
  }
  private: 
  int m_x;
  public: 
  // inline Foo() noexcept = default;
};
```

## 为什么引入[*this]

- [this]引用捕获，在多线程或者异步时，如果捕获的是 this 指针，而原始对象的生命周期结束，可能会导致未定义行为。通过捕获 *this，可以确保 lambda 表达式中使用的对象副本始终有效
- [*this]是值捕获，以避免列出类中每个需要的成员,避免悬挂指针问题
