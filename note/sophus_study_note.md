- [sophus study note](#sophus-study-note)
  - [template type/class](#template-typeclass)
  - [typename 和 constexpr](#typename-和-constexpr)
  - [traits 类型萃取](#traits-类型萃取)
    - [模板特化](#模板特化)
    - [模板偏特化](#模板偏特化)
    - [简单模拟 traits classes](#简单模拟-traits-classes)
    - [sophus traits](#sophus-traits)
  - [enable\_if](#enable_if)
    - [sophus enable\_if example2](#sophus-enable_if-example2)
    - [对函数模板使用 enable\_if](#对函数模板使用-enable_if)

# sophus study note

- code https://github.com/strasdat/Sophus.git
- version: 非模板库版本 commit id: a621ff

## template type/class

```cpp
  /// Returns copy of instance casted to NewScalarType.
  template <class NewScalarType>
  SOPHUS_FUNC SO3<NewScalarType> cast() const {
    return SO3<NewScalarType>(unit_quaternion().template cast<NewScalarType>());
  }
```

对于 templated type/class, 要加上template关键字, 告诉编译器后面跟着的cast 是一个templated method而不是一个成员变量, 让编译器正确识别这部分语法和编译, 如下

```cpp
Vector3<Scalar> const upsilon = upsilon_omega.template head<3>();
Vector3<Scalar> const omega = upsilon_omega.template tail<3>();
Matrix3<Scalar> const J = SO3<Scalar>::leftJacobian(omega);

```

## typename 和 constexpr

```cpp
template <class Derived>
class SO3Base {
 public:
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using QuaternionType =
      typename Eigen::internal::traits<Derived>::QuaternionType;
  using QuaternionTemporaryType = Eigen::Quaternion<Scalar, Options>;

  
  static int constexpr DoF = 3;
  static int constexpr num_parameters = 4;
  static int constexpr N = 3;
  using Transformation = Matrix<Scalar, N, N>;
  using Point = Vector3<Scalar>;
  using HomogeneousPoint = Vector4<Scalar>;
  using Line = ParametrizedLine3<Scalar>;
  using Hyperplane = Hyperplane3<Scalar>;
  using Tangent = Vector<Scalar, DoF>;
  using Adjoint = Matrix<Scalar, DoF, DoF>;

```

```cpp
using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
```

一般情况下, typename 和 class 在C++中等价, 比如如下代码中的class T可以换成typename T

```cpp
template <class T> 
class Test{} ;
```

但是有时候就必须只能用typename. 因为默认情况下,C++语言假定通过作用域运算符访问的名字不是类型。因此, 如果我们希望使用一个模板类型参数的类型成员, 就必须显示告知编译器该名字是一个类型, 就如上面代码所示, 让编译器知道 Eigen::internal::traits<Derived>::Scalar 是一个数据类型

此外, typename 这一性质还经常用在模板函数的返回值定义上, 比如:

```cpp
template<typename T>
typename T::value_type top(const T &c)
{
    if (!c.empty())
        return c.back();
    else
        return typename T::value_type();
}
```

其中函数 top() 期待的入参是一个容器类型, 然后它使用typename指明返回值的类型和入参类型一样; 并且, 在入参c容器中没有元素时, 用typename T::value_type() 生成一个初始值的元素,然后返回。

constexpr 表明这是一个编译期常量.即它的值会在编译期就尝试被计算出来并固定, 提升运行期的执行效率

## traits 类型萃取

- 在 STL 中，容器与算法是分开的，彼此独立设计，容器与算法之间通过迭代器联系在一起。那么，算法是如何从迭代器类中萃取出容器元素的类型的？没错，这正是我们要说的 traits classes 的功能。(类型萃取)
- 迭代器所指对象的类型，称为该迭代器的 `value_type`

这里说一下模板的特化(Template Specialization)和偏特化(Patial Spcialization)

### 模板特化

模板通用定义

```cpp
template<typename T>
struct my_is_void {
    static const bool value = false;
};
```

然后，针对 void 类型的特化

```cpp
template<>
struct my_is_void<void> {
    static const bool value = true;
};
```

output

```cpp
my_is_void<bool> t1;
cout << t1.value << endl;  // 输出0
my_is_void<void> t2; // 当声明 my_is_void<void> t2; 时，使用的是特化版本，故其 value 值为 1
cout << t2.value << endl;  // 输出1
```

### 模板偏特化

```cpp
template<typename T>
struct my_is_pointer {
    static const bool value = false;
};
```

我们对模板参数T进行限制，要求其为一个指针的类型

```cpp
template<typename T>
struct my_is_pointer<T*> {
    static const bool value = true;
};
```

output

```cpp
my_is_pointer<int> p1;
cout << p1.value << endl;  // 输出 0，使用原始模板
my_is_pointer<int*> p2;
cout << p2.value << endl;  // 输出 1，使偏特化模板，因为指定了 int * 类型的参数
```

### 简单模拟 traits classes

```cpp
template<class IterT> 
struct my_iterator_traits {
    typedef typename IterT::value_type value_type;
};
```

> my_iterator_traits 其实就是个类模板，其中包含一个类型的声明。有上面 typename 的基础，相信大家不难理解 typedef typename IterT::value_type value_type; 的含义：将迭代器的value_type 通过 typedef 为 value_type

对于my_iterator_traits，我们再声明一个偏特化版本

```cpp
template<class IterT>
struct my_iterator_traits<IterT*> {
    typedef IterT value_type;
};
```

即如果 my_iterator_traits 的实参为指针类型时，直接使用指针所指元素类型作为 value_type

为了测试 my_iterator_traits 能否正确萃取迭代器元素的类型，我们先编写以下的测试函数

```cpp
void fun(int a) {
    cout << "fun(int) is called" << endl;
}

void fun(double a) {
    cout << "fun(double) is called" << endl;
}

void fun(char a) {
    cout << "fun(char) is called" << endl;
}

my_iterator_traits<vector<int>::iterator>::value_type a;
fun(a);  // 输出 fun(int) is called
my_iterator_traits<vector<double>::iterator>::value_type b;
fun(b);  // 输出 fun(double) is called
my_iterator_traits<char*>::value_type c;
fun(c);  // 输出 fun(char) is called
```

为了便于理解，我们这里贴出 vector 迭代器声明代码的简化版本

```cpp
template <class T, ...>
class vector {
public:
    class iterator {
    public:
        typedef T value_type;
        ...
    };
...
};
```

- my_iterator_traits<vector<int>::iterator>::value_type a: vector<int>::iterator 为vector<int> 的迭代器，该迭代器包含了 value_type 的声明，由 vector 的代码可以知道该迭代器的value_type 即为 int 类型
- my_iterator_traits<vector<int>::iterator> 会采用 my_iterator_traits 的通用版本，即 my_iterator_traits<vector<int>::iterator>::value_type 使用 typename IterT::value_type 这一类型声明，这里 IterT 为 vector<int>::iterator，故整个语句萃取出来的类型为 int 类型,对 double 类型的 vector 迭代器的萃取也是类似的过程
- my_iterator_traits<char*>::value_type 则使用 my_iterator_traits 的偏特化版本，直接返回 char 类型

### sophus traits

Sophus库是在Eigen库为基础的增强, 在Eigen库的发布版本源码中是没有SO3的相关数据类型的定义的,故Sophus定义了这些traits

```cpp
namespace Sophus {
template <class Scalar_, int Options = 0>
class SO3;
using SO3d = SO3<double>;
using SO3f = SO3<float>;
}  // namespace Sophus

namespace Eigen {
namespace internal {

template <class Scalar_, int Options_>
struct traits<Sophus::SO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Eigen::Quaternion<Scalar, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::SO3<Scalar_>, Options_>>
    : traits<Sophus::SO3<Scalar_, Options_>> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar>, Options>;
};

template <class Scalar_, int Options_>
struct traits<Map<Sophus::SO3<Scalar_> const, Options_>>
    : traits<Sophus::SO3<Scalar_, Options_> const> {
  static constexpr int Options = Options_;
  using Scalar = Scalar_;
  using QuaternionType = Map<Eigen::Quaternion<Scalar> const, Options>;
};
}  // namespace internal
}  // namespace Eigen
```

- 在`Eigen::internal`命名空间中,增加了一个新的traits.其中的Sophus::SO3<Scalar_, Options_>, 表明这个 traits 是针对哪个数据类型的特化
- 在这个traits类里面, 又定义了两个新的数据类型, 分别是Scalar, QuaternionType
- 在有了如上定义之后, 以后如果要使用到这两个新定义的数据类型, 则可以采用如下代码形式

```cpp
template<typename Derived>
class T {
public:
    using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
    using QuaternionType = typename Eigen::internal::traits<Derived>::QuaternionType;
    //...
};
```

- 定义新的数据类型Scalar, QuaternionType, 即把traits中的类型暴露出来并别名化的过程
- 当出现以上代码时, Scalar也并不一定会指向上面为SO3所定义的traits. 因为Eigen库中为旋转向量、旋转矩阵等等都定义了对应的traits, 还是要看Derived是什么类型. 在Sophus库中, 其中的模板参数Derrived往往会被特化为Sophus::SO3类型, 这时候编译器就会使用上面traits的定义了

## enable_if

- enable_if 带两个模板元参数, 第一个是 bool , 第二个为 typename, 它的意思就是当第一个参数为 true 时, 第二个参数有效, 否则没有定义(然后可以触发编译器报错). 主要用在模板元变成场景中
- enable_if 是通过结构体模板来实现

```cpp
template<bool B, typename T=void>
struct enable_if {};

template<typename T>
struct enable_if<true, T> {
    typedef T type;
}
```

只有当第一个模板参数 B 为true时, enable_if 会包含一个 type=T 的成员, 否则就没有该成员(void)

- sophus中的enable_if

```cpp
   /// Returns closed SO2 given arbitrary 2x2 matrix.
   ///
   template <class S = Scalar>
   static SOPHUS_FUNC enable_if_t<std::is_floating_point<S>::value, SO2>
   fitToSO2(Transformation const& R) {
     return SO2(makeRotationMatrix(R));
   }
```

- 定义了一个函数 fitToSO2, 入参是一个 2x2 矩阵(参见Transformation的定义):`using Transformation = Matrix<Scalar, N, N>;`
- 矩阵的数据类型为 S = Scalar, 使用的是模板参数
- 函数的返回类型定义时, 使用了 enable_if_t 上面代码
- enable_if_t<std::is_floating_point<S>::value, SO2> 的意思是
  - 如果数据类型 S 是浮点数时, 则函数的返回值为 SO2
  - 否则函数返回值没有定义. 其中 enable_if_t 的定义

```cpp
template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;
```

为什么要用 enable_if?

- 如果我们是希望只有当入参为浮点数类型时, fitToSO2 才有返回的数据类型 SO2, 当传入其他类型数据时, fitToSO2 不返回 SO2, 而是返回其他类型数据, 或者如 Sophus 中这么定义的, 直接让编译器报错.
- 此时, 就可以使用 enable_if 根据入参的类型不同, 来控制函数的返回值类型, 达到所需的效果.

### sophus enable_if example2

```cpp
 template <class Ptr>
 class Pretty<Ptr, enable_if_t<std::is_pointer<Ptr>::value>> {
  public:
   static std::string impl(Ptr ptr) {
     std::stringstream sstr;
     sstr << std::intptr_t(ptr);
     return sstr.str();
   }
 };
```

### 对函数模板使用 enable_if

在其他场合, 例如一个函数模板的有多个模板参数,且需要对其中一个模板参数进行特化, 其他模板参数不进行特化时, 怎么处理?  由于函数模板不能支持偏特化, 即模板参数只有全特化, 此时就可以使用 enable_if 对其中一个模板参数进行判断,针对每种条件都写一个函数实现体, 间接的实现函数特化

```cpp
 template <std::size_t k, class T, class... Ts>
 typename std::enable_if<k==0, typename element_type_holder<0, T, Ts...>::type&>::type
 get(tuple<T, Ts...> &t) {
       return t.tail; 
 }

 template <std::size_t k, class T, class... Ts>
 typename std::enable_if<k!=0, typename element_type_holder<k, T, Ts...>::type&>::type
 get(tuple<T, Ts...> &t) {
       tuple<Ts...> &base = t;
       return get<k-1>(base); 
 }
```
