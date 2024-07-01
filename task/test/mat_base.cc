#include <chrono>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

void image_test() {
  Mat check_image = imread("../data/lena.png", cv::IMREAD_GRAYSCALE);

  // 判断图像文件是否正确读取
  if (check_image.data == nullptr) { //数据不存在,可能是文件不存在
    return;
  }

  Mat image(10000, 10000, CV_8UC3);
  // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
  // 使用 std::chrono 来给算法计时
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // 用cv::Mat::ptr获得图像的行指针
    unsigned char *row_ptr =
        image.ptr<unsigned char>(y); // row_ptr是第y行的头指针
    for (size_t x = 0; x < image.cols; x++) {
      // 访问位于 x,y 处的像素
      unsigned char *data_ptr =
          &row_ptr[x * image.channels()]; // data_ptr 指向待访问的像素数据
      // 输出该像素的每个通道,如果是灰度图就只有一个通道
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
      }
    }
  }

  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "1. 遍历图像用时：" << time_used.count() << " 秒。" << endl;

  chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
  if (image.channels() == 1) {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        unsigned char val = image.at<uchar>(i, j);
      }
    }
  } else if (image.channels() == 3) {
    for (int i = 0; i < image.rows; i++) {
      for (int j = 0; j < image.cols; j++) {
        unsigned char b = image.at<Vec3b>(i, j)[0];
        unsigned char g = image.at<Vec3b>(i, j)[1];
        unsigned char r = image.at<Vec3b>(i, j)[2];
      }
    }
  }
  chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
  chrono::duration<double> time_used_2 =
      chrono::duration_cast<chrono::duration<double>>(t4 - t3);
  cout << "2. 遍历图像用时：" << time_used_2.count() << " 秒。" << endl;
}

void type_convert() {
  cv::Mat mat; // matrix head
  mat = (cv::Mat_<double>(3, 3) << 999.0, 324.0, 110.0, -1.0, -222.0, 813.0,
         -671.0, 3.0, 1.0);

  Mat convert_mat;
  mat.convertTo(convert_mat, CV_8UC1);
  cout << mat << "\n" << convert_mat << endl;
}

void multiplication_operator() {
  // multiplication
  Mat m1 = Mat::eye(2, 3, CV_32F);
  Mat m2 = Mat::ones(3, 2, CV_32F);
  cout << "m1  = " << endl << m1 << endl << endl;
  cout << "m2  = " << endl << m2 << endl << endl;
  // Scalar by matrix
  cout << "\nm1.*2 = \n" << m1 * 2 << endl;
  // matrix per element multiplication
  cout << "\n(m1+2).*(m1+3) = \n" << (m1 + 1).mul(m1 + 3) << endl;
  // Matrix multiplication
  cout << "\nm1*m2 = \n" << m1 * m2 << endl;
}

void transpose() {
  Mat m1 = Mat::eye(2, 3, CV_32F);
  Mat m1t = m1.t();
  cout << "m1  = " << endl << m1 << endl << endl;
  cout << "m1t  = " << endl << m1t << endl << endl;
}

void inverse() {
  Mat me = Mat::eye(5, 5, CV_32F);
  Mat me_inv = me.inv();
  cout << "me  = " << endl << me << endl << endl;
  cout << "me_inv = " << endl << me_inv << endl << endl;
}

int main() {
  // cv::Mat including matrix head and matrix pointer to pixel data
  // 矩阵头
  // 包括数字图像的矩阵尺寸、存储方法、存储地址和引用次数等，矩阵头的大小是一个常数，不会随着图像的大小而改变
  // OpenCV使用了引用次数，当进行图像复制和传递时，不再复制整个Mat数据，而只是复制矩阵头和指向像素矩阵的指针

  cv::Mat mat_a; // matrix head
  mat_a = (cv::Mat_<double>(3, 3) << 1.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0, 0.0,
           1.0); // matrix pointer to data

  cv::Mat mat_b = mat_a;

  // 1. construction
  //   type
  // CV_8UC1、CV_8UC2、CV_8UC3	uchar
  // CV_8SC1、CV_8SC2、CV_8SC3	char
  // CV_16UC1、CV_16UC2、CV_16UC3	ushort
  // CV_16SC1、CV_16SC2、CV_16SC3	short
  // CV_32SC1、CV_32SC2、CV_32SC3	int
  // CV_32FC1、CV_32FC2、CV_32FC3	float
  // CV_64FC1、CV_64FC2、CV_64FC3	double

  // 1. cv::Mat::Mat(int rows,int cols,int type) rows 行；cols 列
  Mat src1(3, 4, CV_32FC3);

  // 2. cv::Mat::Mat(Size size,int type ) size::Size(cols,rows)
  Mat src2(Size(3, 4), CV_32FC3);
  cout << "1. src1.rows=" << src1.rows << " src1.cols=" << src1.cols << endl;
  cout << "2. src2.rows=" << src2.rows << " src2.cols=" << src2.cols << endl;
  cout << "src1.size=" << src1.size() << endl
       << "src2.size=" << src2.size() << endl;

  // 3. cv::Mat::Mat(int ndims,const int *  sizes,int type,const Scalar& s)
  Mat src3(300, 400, CV_8UC3, Scalar(255, 255, 255));
  cout << "3. src3.rows=" << src3.rows << " src3.cols=" << src3.cols << endl;

  // 4. cv::Mat::Mat(const Mat & m)
  Mat src4(src3);
  cout << "4. src4.rows=" << src4.rows << " src4.cols=" << src4.cols << endl;

  // 5. Mat(Size size, int type, const Scalar& s)
  Mat src5(Size(400, 300), CV_8UC3, Scalar(255, 255, 255));
  cout << "5. src5.rows=" << src5.rows << " src5.cols=" << src5.cols << endl;

  // 6.  Mat(int rows, int cols, int type, void* data, size_t step=AUTO_STEP)
  // Mat(Size size, int type, void* data, size_t step=AUTO_STEP)
  Mat lena = imread("../data/lena.png", cv::IMREAD_GRAYSCALE);
  Mat src6(mat_a.rows, mat_a.cols, CV_64FC1, mat_a.data, 0);
  cout << "6. src6: \n" << src6 << endl;

  // 7. Mat(const Mat& m, const Range& rowRange, const Range& colRange)
  // Range::all()
  Mat src7(lena, cv::Range(50, 300), cv::Range(100, 150));

  // 8. Mat::Mat(const Mat& m, const Rect& roi) Rect(col, row, col+width,
  // row_height)
  Mat srcROI(lena, Rect(0, 200, 300, 100));

  // =====================================================
  // depth 对于16位有符号的3通道数组，该方法返回CV_16S
  cout << "lena depth: " << lena.depth() << ", mat_a depth: " << mat_a.depth()
       << endl;

  // void Mat::create(int rows, int cols, int type)
  // void Mat::create(Size size, int type)
  // void Mat::create(int ndims, const int* sizes, inttype)

  // 矩阵元素大小（以字节为单位）。size_t Mat::elemSize() const
  cout << "lena elesz: " << lena.elemSize()
       << ", mat_a elesz: " << mat_a.elemSize() << endl;

  // 以字节为单位返回每个矩阵元素通道的大小
  cout << "lena elemSize1: " << lena.elemSize1()
       << ", mat_a elemSize1: " << mat_a.elemSize1() << endl;

  // Mat::step1
  cout << "lena step1: " << lena.step1() << ", mat_a step1: " << mat_a.step1()
       << endl;

  // 返回指定矩阵行的指针
  // uchar* Mat::ptr(int i=0)
  // const uchar* Mat::ptr(int i=0) const
  // template<typename _Tp> _Tp* Mat::ptr(inti=0)
  // template<typename _Tp> const _Tp*Mat::ptr(int i=0) const

  // copy
  Mat copy_mat_a = mat_a.clone();
  Mat copy_mat_b;
  mat_a.copyTo(copy_mat_b);
  cout << "copy_mat_a: " << copy_mat_a << "copy_mat_b: " << copy_mat_b << endl;

  image_test();
  type_convert();
  multiplication_operator();
  transpose();
  inverse();
  return 0;
}
