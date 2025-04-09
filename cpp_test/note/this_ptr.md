- [C++ `this` pointer](#c-this-pointer)

# C++ `this` pointer

this 指针是一个特殊的指针，它指向调用成员函数的那个对象。在非静态成员函数内部，你可以使用 this 指针访问调用对象的成员。本质上，this 指针是编译器隐式提供的，我们并不需要定义它，但我们可以在成员函数内部使用它

- this 指针可以在类的所有非静态成员函数中使用，包括构造函数和析构函数
- 实现链式调用

```cpp
class Box {
    int length, width;  // 定义私有的整型成员变量length和width
public:
    Box& setLength(int length) {  // 返回一个指向当前对象的引用
        this->length = length;  // 将传递的参数 length 赋给成员变量 length
        return *this;  // 返回指向当前对象的引用
    }
 
    Box& setWidth(int width) {  // 返回一个指向当前对象的引用
        this->width = width;  // 将传递的参数 width 赋给成员变量 width
        return *this;  // 返回指向当前对象的引用
    }
 
    void display() {  // 定义成员函数 display
        std::cout << "Length: " << length << ", Width: " << width << std::endl;  // 输出成员变量 length 和 width 的值
    }
};
 
// 使用示例
Box b;
b.setLength(10).setWidth(5).display();  // 链式调用 setLength, setWidth, display 函数显示结果
```

- 拷贝构造函数和赋值操作符

this 指针在拷贝构造函数和赋值操作符中也扮演着重要的角色。在这些函数中，我们通常需要检查传入的对象是否就是当前对象（即，是否是自我赋值）。如果是，则应避免进行可能导致错误的自我赋值

```cpp
class Box {
    int* data;  // 定义私有指针成员变量 data
 
public:
    // 赋值运算符重载函数
    Box& operator=(const Box& other) {
        if (this != &other) {  // 防止自赋值的情况
            delete[] data;  // 释放旧内存
            data = new int[10];  // 重新分配内存
            std::copy(other.data, other.data + 10, data);  // 拷贝数据
        }
        return *this;  // 返回一个指向当前对象的引用
    }
};
```

- const 成员函数中 this 指针的类型和意义 在 const 成员函数中，this 指针的类型是指向 const 类型的指针。这意味着你不能通过 this 指针修改对象的状态

```cpp
class Box {
    int length;
 
public:
    int getLength() const { // 常成员函数
        // this->length = 10;  // 错误：不能在常成员函数中修改成员变量 this 指针的类型是 const Box*，所以你不能通过 this 指针来修改 length
        return length;  // 返回成员变量的值
    }
};
```

- const 成员函数是一种保证对象状态不变性的重要机制。当你将一个成员函数声明为 const，你就是在承诺这个函数不会修改对象的状态

```cpp
class Box {
    int length;
 
public:
    Box(int length) : length(length) {}  // 构造函数，初始化成员变量 length
 
    void increaseLength(int increment) {  // 增加盒子长度的函数
        length += increment;  // 修改对象的状态
    }
 
    int getLength() const {  // 获取盒子长度的函数，常成员函数
        return length;  // 不修改对象的状态，只返回成员变量的值
    }
};
```

- 在多线程环境中，每个线程都有独立的 this 指针，因此无法跨线程传递 this 指针。在多线程编程中，this 指针常用于处理多线程同步和数据竞争问题
- 在构造函数和析构函数中，对 this 指针的使用有限制。在构造函数中，this 指针不能用于访问未初始化的成员变量；在析构函数中，使用 this 指针可能导致未定义的行为

```cpp
class Box {
    int* data;
 
public:
    Box() {
        data = new int[10];
        // 避免使用 'this' 进行重要操作，因为对象没有完全构造完成
    }
 
    ~Box() {
        delete[] data;
        // 对象正在被析构，进一步使用 'this' 可能会导致未定义行为
    }
};
```

- 避免返回将要析构对象的 this 指针

```cpp
class Box {
    int data;
 
public:
    Box(int value) : data(value) {}
 
    Box* getDataPtr() {
        return this;
    }
};
 
Box* badFunc() {
    return Box(10).getDataPtr();  // 返回指向已经被析构的对象的指针
}
 
// 使用示例
Box* p = badFunc();  // 'p' 是一个悬垂指针（dangling pointer）
```

在上述代码中，badFunc 函数返回的是一个将要被析构的临时对象的 this 指针，这会导致 p 成为一个悬垂指针。为了避免这种情况，我们应该尽量避免在成员函数中返回 this 指针，或者确保返回的 this 指针指向的对象在函数返回后仍然存在


参考文章

- [C++中的 `this` 指针](https://blog.csdn.net/crr411422/article/details/131063469)
