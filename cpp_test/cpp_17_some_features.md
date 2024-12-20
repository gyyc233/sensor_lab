
## CPP17 Some Features

1. 构造函数模板推导

```cpp
pair<int, double> p(1, 2.2); // before c++17
pair p(1, 2.2); // c++17 自动推导
vector v = {1, 2, 3}; // c++17
```

2. 结构化绑定 Structured Bindings 允许将元组或对的元素解包到单独的变量中

```cpp
std::tuple<int, double> func() {
    return std::tuple(1, 2.2);
}

int main() {
    auto[i, d] = func(); //是C++11的tie吗？更高级
    cout << i << endl;
    cout << d << endl;
}

//==========================
void f() {
    map<int, string> m = {
      {0, "a"},
      {1, "b"},  
    };
    for (const auto &[i, s] : m) {
        cout << i << " " << s << endl;
    }
}

// ====================
int main() {
    std::pair a(1, 2.3f);
    auto[i, f] = a;
    cout << i << endl; // 1
    cout << f << endl; // 2.3f
    return 0;
}
```

结构化绑定还可以改变对象的值，使用引用即可：

```cpp
// 进化，可以通过结构化绑定改变对象的值
int main() {
    std::pair a(1, 2.3f);
    auto& [i, f] = a;
    i = 2;
    cout << a.first << endl; // 2 
}
```

注意结构化绑定不能应用于constexpr

```cpp
constexpr auto[x, y] = std::pair(1, 2.3f); // compile error, C++20可以
```

结构化绑定不止可以绑定pair和tuple，还可以绑定数组和结构体等

```cpp
// 需要实现相关的tuple_size和tuple_element和get<N>方法。
class Entry {
public:
    void Init() {
        name_ = "name";
        age_ = 10;
    }

    std::string GetName() const { return name_; }
    int GetAge() const { return age_; }
private:
    std::string name_;
    int age_;
};

template <size_t I>
auto get(const Entry& e) {
    if constexpr (I == 0) return e.GetName();
    else if constexpr (I == 1) return e.GetAge();
}

namespace std {
    template<> struct tuple_size<Entry> : integral_constant<size_t, 2> {};
    template<> struct tuple_element<0, Entry> { using type = std::string; };
    template<> struct tuple_element<1, Entry> { using type = int; };
}

int main() {
    Entry e;
    e.Init();
    auto [name, age] = e;
    cout << name << " " << age << endl; // name 10
    return 0;
}
```

实现自定义类的结构化绑定

```cpp
// 需要实现相关的tuple_size和tuple_element和get<N>方法。
class Entry {
public:
    void Init() {
        name_ = "name";
        age_ = 10;
    }

    std::string GetName() const { return name_; }
    int GetAge() const { return age_; }
private:
    std::string name_;
    int age_;
};

template <size_t I>
auto get(const Entry& e) {
    if constexpr (I == 0) return e.GetName();
    else if constexpr (I == 1) return e.GetAge();
}

namespace std {
    template<> struct tuple_size<Entry> : integral_constant<size_t, 2> {};
    template<> struct tuple_element<0, Entry> { using type = std::string; };
    template<> struct tuple_element<1, Entry> { using type = int; };
}

int main() {
    Entry e;
    e.Init();
    auto [name, age] = e;
    cout << name << " " << age << endl; // name 10
    return 0;
}
```

3. if-switch语句初始化

```cpp
// if (init; condition)

if (int a = GetValue()); a < 101) {
    cout << a;
}

string str = "Hi World";
if (auto [pos, size] = pair(str.find("Hi"), str.size()); pos != string::npos) {
    std::cout << pos << " Hello, size is " << size;
}
```

4. 内联变量

我们印象中C++类的静态成员变量在头文件中是不能初始化的，但是有了内联变量，就可以达到此目的：

```cpp
// header file
struct A {
    static const int value;  
};
inline int const A::value = 10;

// ==========或者========
struct A {
    inline static const int value = 10;
}
```

5. 折叠表达式

简化了在变长模板参数包中应用二元运算符的过程

```cpp
template <typename ... Ts>
auto sum(Ts ... ts) {
    return (ts + ...);
}
int a {sum(1, 2, 3, 4, 5)}; // 15
std::string a{"hello "};
std::string b{"world"};
cout << sum(a, b) << endl; // hello world
```

6. constexpr lambda表达式

C++17前lambda表达式只能在运行时使用，C++17引入了constexpr lambda表达式，可以用于在编译期进行计算。

```cpp
int main() { // c++17可编译
    constexpr auto lamb = [] (int n) { return n * n; };
    static_assert(lamb(3) == 9, "a");
}
```

注意：constexpr函数有如下限制：

函数体不能包含汇编语句、goto语句、label、try块、静态变量、线程局部存储、没有初始化的普通变量，不能动态分配内存，不能有new delete等，不能虚函数。

7. __has_include预处理表达式

可以判断是否有某个头文件，代码可能会在不同编译器下工作，不同编译器的可用头文件有可能不同，所以可以使用此来判断：

```cpp
#include <iostream>
 
int main()
{
#if __has_include(<cstdio>)
    printf("c program");
#endif
 
#if __has_include("iostream")
    std::cout << "c++ program" << std::endl;
#endif
 
    return 0;
}
```

如果相应的#include命令有效，那么_has_include(…)中的条件的值为1 (true)

8. 在lambda表达式用*this捕获对象副本

lambda表达式中访问类的对象成员变量需要捕获this，但是这里捕获的是this指针，指向的是对象的引用，正常情况下可能没问题，但是如果多线程情况下，函数的作用域超过了对象的作用域，对象已经被析构了，还访问了成员变量，就会有问题

```cpp
struct A {
    int a;
    void func() {
        auto f = [this] {
            cout << a << endl;
        };
        f();
    }  
};
int main() {
    A a;
    a.func();
    return 0;
}
```

C++17增加了新特性，捕获*this，不持有this指针，而是持有对象的拷贝，这样生命周期就与对象的生命周期不相关

```cpp
struct A {
    int a;
    void func() {
        auto f = [*this] { // 这里
            cout << a << endl;
        };
        f();
    }  
};
int main() {
    A a;
    a.func();
    return 0;
}
```

9.  新增Attribute

```cpp
[[carries_dependency]] 让编译期跳过不必要的内存栅栏指令
[[noreturn]] 函数不会返回
[[deprecated]] 函数将弃用的警告

[[noreturn]] void terminate() noexcept;
[[deprecated("use new func instead")]] void func() {}
```

C++17又新增了三个

- [[fallthrough]]，用在switch中提示可以直接落下去，不需要break，让编译器忽略警告

```cpp
switch (i) {
    case 1:
        xxx; // warning
    case 2:
        xxx; 
        [[fallthrough]];      // 警告消除
    case 3:
        xxx;
       break;
}
```
- [[nodiscard]] ：提示编译器修饰的内容可能没有使用，避免产生警告

```cpp
[[nodiscard]] int func();
void F() {
    func(); // warning 没有处理函数返回值
}
```

- [[maybe_unused]] ：提示编译器修饰的内容可能暂时没有使用，避免产生警告

```cpp
void func1() {}
[[maybe_unused]] void func2() {} // 警告消除
void func3() {
    int x = 1;
    [[maybe_unused]] int y = 2; // 警告消除
}
```

10. 字符串转换

新增from_chars函数和to_chars函数

```cpp
#include <charconv>

int main() {
    const std::string str{"123456098"};
    int value = 0;
    const auto res = std::from_chars(str.data(), str.data() + 4, value);
    if (res.ec == std::errc()) {
        cout << value << ", distance " << res.ptr - str.data() << endl;
    } else if (res.ec == std::errc::invalid_argument) {
        cout << "invalid" << endl;
    }
    str = std::string("12.34);
    double val = 0;
    const auto format = std::chars_format::general;
    res = std::from_chars(str.data(), str.data() + str.size(), value, format);

    str = std::string("xxxxxxxx");
    const int v = 1234;
    res = std::to_chars(str.data(), str.data() + str.size(), v);
    cout << str << ", filled " << res.ptr - str.data() << " characters \n";
    // 1234xxxx, filled 4 characters
}
```

11. std::variant

C++17增加std::variant实现类似union的功能，但却比union更高级，举个例子union里面不能有string这种类型，但std::variant却可以，还可以支持更多复杂类型，如map等

```cpp
int main() { // c++17可编译
    std::variant<int, std::string> var("hello");
    cout << var.index() << endl;
    var = 123;
    cout << var.index() << endl;

    try {
        var = "world";
        std::string str = std::get<std::string>(var); // 通过类型获取值
        var = 3;
        int i = std::get<0>(var); // 通过index获取对应值
        cout << str << endl;
        cout << i << endl;
    } catch(...) {
        // xxx;
    }
    return 0;
}
```

注意：一般情况下variant的第一个类型一般要有对应的构造函数，否则编译失败：

```cpp
struct A {
    A(int i){}  
};
int main() {
    std::variant<A, int> var; // 编译失败
}
```



如何避免这种情况呢，可以使用std::monostate来打个桩，模拟一个空状态。

`std::variant<std::monostate, A> var; // 可以编译成功`

12. std::optional

我们有时候可能会有需求，让函数返回一个对象，如下：

```cpp
struct A {};
A func() {
    if (flag) return A();
    else {
        // 异常情况下，怎么返回异常值呢，想返回个空呢
    }
}
```

有一种办法是返回对象指针，异常情况下就可以返回nullptr啦，但是这就涉及到了内存管理，也许你会使用智能指针，但这里其实有更方便的办法就是std::optional

```cpp
std::optional<int> StoI(const std::string &s) {
    try {
        return std::stoi(s);
    } catch(...) {
        return std::nullopt;
    }
}

void func() {
    std::string s{"123"};
    std::optional<int> o = StoI(s);
    if (o) {
        cout << *o << endl;
    } else {
        cout << "error" << endl;
    }
}
```

13. std::any

any可以存储任何类型的单个值，见代码：

```cpp
int main() { // c++17可编译
    std::any a = 1;
    cout << a.type().name() << " " << std::any_cast<int>(a) << endl;
    a = 2.2f;
    cout << a.type().name() << " " << std::any_cast<float>(a) << endl;
    if (a.has_value()) {
        cout << a.type().name();
    }
    a.reset();
    if (a.has_value()) {
        cout << a.type().name();
    }
    a = std::string("a");
    cout << a.type().name() << " " << std::any_cast<std::string>(a) << endl;
    return 0;
}
```

14. std::apply

std::apply可以将tuple展开作为函数的参数传入

```cpp
int add(int first, int second) { return first + second; }

auto add_lambda = [](auto first, auto second) { return first + second; };

int main() {
    std::cout << std::apply(add, std::pair(1, 2)) << '\n';
    std::cout << add(std::pair(1, 2)) << "\n"; // error
    std::cout << std::apply(add_lambda, std::tuple(2.0f, 3.0f)) << '\n';
}
```

15. std::make_from_tuple

使用make_from_tuple可以将tuple展开作为构造函数参数

```cpp
struct Foo {
    Foo(int first, float second, int third) {
        std::cout << first << ", " << second << ", " << third << "\n";
    }
};
int main() {
   auto tuple = std::make_tuple(42, 3.14f, 0);
   std::make_from_tuple<Foo>(std::move(tuple));
}
```

16. std::string_view

通常我们传递一个string时会触发对象的拷贝操作，大字符串的拷贝赋值操作会触发堆内存分配，很影响运行效率，有了string_view就可以避免拷贝操作，平时传递过程中传递string_view

```cpp
void func(std::string_view stv) { cout << stv << endl; }

int main(void) {
    std::string str = "Hello World";
    std::cout << str << std::endl;

    std::string_view stv(str.c_str(), str.size());
    cout << stv << endl;
    func(stv);
    return 0;
}
```

17. as_const
C++17使用as_const可以将左值转成const类型

```cpp
std::string str = "str";
const std::string& constStr = std::as_const(str);
```

18. file_system

```cpp
namespace fs = std::filesystem;
fs::create_directory(dir_path);
fs::copy_file(src, dst, fs::copy_options::skip_existing);
fs::exists(filename);
fs::current_path(err_code);
```

19. std::shared_mutex

std::shared_mutex 是一种互斥锁类型，允许多个读线程同时访问资源，或者一个写线程独占访问资源，但不允许两者同时进行。它提供了共享所有权语义的线程同步机制，特别适用于多个线程并发读取数据且偶尔写入数据的场景。

主要特性：
- 多个读取者： 多个线程可以同时获取共享锁，允许并发读取。
- 独占写入者： 只有一个线程可以获取独占锁，防止其他任何线程（无论是读还是写）访问资源。

成员函数：
- lock_shared()： 获取共享锁。如果另一个线程持有独占锁，则阻塞。
- try_lock_shared()： 尝试获取共享锁而不阻塞。成功返回 true，否则返回 false。
- unlock_shared()： 释放共享锁。
- lock()： 获取独占锁。直到没有其他锁被持有时才会继续执行。
- try_lock()： 尝试获取独占锁而不阻塞。成功返回 true，否则返回 false。
- unlock()： 释放独占锁。

```cpp
#include <iostream>
#include <shared_mutex>
#include <thread>
#include <vector>

class Data {
public:
    void read(int id) {
        std::shared_lock<std::shared_mutex> lock(mtx);
        // 读取临界区
        std::cout << "Reader " << id << " is reading.\n";
    }

    void write(int id) {
        std::unique_lock<std::shared_mutex> lock(mtx);
        // 写入临界区
        std::cout << "Writer " << id << " is writing.\n";
    }

private:
    std::shared_mutex mtx;
};

void reader(Data& data, int id) {
    while (true) {
        data.read(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void writer(Data& data, int id) {
    while (true) {
        data.write(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main() {
    Data data;
    std::vector<std::thread> readers;
    std::vector<std::thread> writers;

    for (int i = 0; i < 3; ++i) {
        readers.emplace_back(reader, std::ref(data), i);
    }

    for (int i = 0; i < 2; ++i) {
        writers.emplace_back(writer, std::ref(data), i);
    }

    for (auto& t : readers) {
        t.join();
    }

    for (auto& t : writers) {
        t.join();
    }

    return 0;
}
```

在这个示例中：

多个读取者线程可以使用 std::shared_lock 并发访问 read 函数。
只有一个写入者线程可以使用 std::unique_lock 访问 write 函数，并确保与任何读取者或其他写入者互斥。
这种设置确保了高效的并发访问模式，同时保持数据完整性。
