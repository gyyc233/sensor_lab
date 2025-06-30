#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// git@github.com:dcanelhas/sim2-alignment.git

#define EPS                                                                    \
  1e-4 // Threshold value on the change in parameters as a termination
       // criteria(中止标准).
#define HUBER_LOSS                                                             \
  0.80 // Huber loss function parameter, to redunce the influence of outliers
       // Huber 损失函数参数，以消除异常值的影响
#define PAUSE_BETWEEN true // should we pause between each image pyramid?

// bilinear interpol for sub pixel
template <class T> T Interpolate(cv::Mat &image, float y, float x) {
  float xd, yd;
  float k1 = modff(x, &xd); // 将x分为整数和小数部分 xd整数 k1小数
  float k2 = modff(y, &yd);
  int xi = int(xd);
  int yi = int(yd);

  int f1 = xi < image.rows - 1; // Check that pixels to the right
  int f2 = yi < image.cols - 1; // and to down direction exist.

  // 获取亚像素(y,x)附近的四个像素
  T px1 = image.at<T>(yi, xi);         // 左上角
  T px2 = image.at<T>(yi, xi + 1);     // 右上角
  T px3 = image.at<T>(yi + 1, xi);     // 左下角
  T px4 = image.at<T>(yi + 1, xi + 1); // 右下角

  // Interpolate pixel intensity.
  // 根据距离权重对四个邻近像素进行加权平均，得到最终的插值结果
  T interpolated_value =
      (1.0 - k1) * (1.0 - k2) * px1 + (f1 ? (k1 * (1.0 - k2) * px2) : 0) +
      (f2 ? ((1.0 - k1) * k2 * px3) : 0) + ((f1 && f2) ? (k1 * k2 * px4) : 0);

  return interpolated_value;
}

/// @brief images alignment via LK flow inverse Compositional
/// @param source seconde image frame
/// @param target first image frame
/// @param max_iterations max iteration number
/// @param transformation output transformation
void Align(cv::Mat &source, cv::Mat &target, int max_iterations,
           Eigen::Matrix3f &transformation) {
  // clang-format off
  // Find the 2-D similarity transform that best aligns the two images (uniform scale, rotation and translation)
  // 找到最能对齐两张图像的 2-D 相似性变换（均匀缩放、旋转和平移）
  cv::Mat debug;
  
  cv::Mat source_gradient_row;    // Gradient of I in X direction. x(行)方向图像梯度
  cv::Mat source_gradient_col;    // Gradient of I in Y direction. y(列)方向图像梯度
  cv::Mat steepest_descent;       // Steepest descent images. 最速下降图像   

  // Here we will store matrices.
  Eigen::Matrix3f W; // Current value of warp W(x,p)
  Eigen::Matrix3f dW; // Warp update.
  Eigen::Vector3f X; // Point in coordinate frame of source. source 图像坐标系中的点
  Eigen::Vector3f Z; // Point in coordinate frame of target. target 图像坐标系中的点

  Eigen::Matrix4f H; // Approximate Hessian. 近似hessian matrix
  Eigen::Vector4f b; // Vector in the right side of the system of linear equations. 线性方程组右侧的 Vector
  Eigen::Vector4f delta_p; // Parameter update value.
  
  // Create images.
  source_gradient_row = cv::Mat(source.rows, source.cols, CV_32FC1); // source image rows gradient
  source_gradient_col = cv::Mat(source.rows, source.cols, CV_32FC1); // source image cols gradient
  steepest_descent =    cv::Mat(source.rows, source.cols, CV_32FC4); // 4 通道

  //The "magic number" appearing at the end in the following is simply the inverse 
  //of the absolute sum of the weights in the matrix representing the Scharr filter.
  cv::Scharr(source, source_gradient_row, -1, 0, 1, 1.0/32.0); // 行方向scharr图像梯度
  cv::Scharr(source, source_gradient_col, -1, 1, 0, 1.0/32.0); // 列方向scharr图像梯度

  H = Eigen::Matrix4f::Zero();
  float h00 = 0.0, h01 = 0.0, h02 = 0.0, h03 = 0.0; 
  float h10 = 0.0, h11 = 0.0, h12 = 0.0, h13 = 0.0; 
  float h20 = 0.0, h21 = 0.0, h22 = 0.0, h23 = 0.0; 
  float h30 = 0.0, h31 = 0.0, h32 = 0.0, h33 = 0.0;

  // estimate source image hessian matrix
  #pragma omp parallel for \
  reduction(+:h00,h01,h02,h03,h10,h11,h12,h13,h20,h21,h22,h23,h30,h31,h32,h33)
  for(int row=0; row<source.rows; row++)//
  {
    #pragma unroll
    for(int col=0; col<source.cols; col++) //
    {
      // Evaluate image gradient 像素梯度
      Eigen::Matrix<float,1,2> image_jacobian;
      image_jacobian << source_gradient_row.at<float>(row,col),
                        source_gradient_col.at<float>(row,col);

      // printf("image jacobian = %f, %f", image_jacobian(0),image_jacobian(1));

      // 定义了图像坐标相对于变换参数（平移 tx、ty，旋转 θ，缩放 s）的导数,这里旋转定为0，缩放定为1
      Eigen::Matrix<float,2,4> warp_jacobian;
      warp_jacobian <<  1, 0 , row,  row,
                        0, 1 , -col, col;

      // 将图像梯度与变换雅可比矩阵相乘，得到当前像素点在变换参数空间中的“最速下降方向”
      Eigen::Vector4f Jacobian = (image_jacobian*warp_jacobian).transpose();                          

      for(int dim = 0; dim<4; ++dim)
      steepest_descent.at<cv::Vec4f>(row, col)[dim] = Jacobian(dim);

      // 所有像素的 Hpart 累加起来形成整个 Hessian 矩阵
      Eigen::Matrix4f Hpart = Jacobian*Jacobian.transpose(); // 像素的 hessian matrix

      h00+=Hpart(0,0); h01+=Hpart(0,1); h02+=Hpart(0,2); h03+=Hpart(0,3); 
      h10+=Hpart(1,0); h11+=Hpart(1,1); h12+=Hpart(1,2); h13+=Hpart(1,3); 
      h20+=Hpart(2,0); h21+=Hpart(2,1); h22+=Hpart(2,2); h23+=Hpart(2,3); 
      h30+=Hpart(3,0); h31+=Hpart(3,1); h32+=Hpart(3,2); h33+=Hpart(3,3); 
    }
  }

//This is the "inverse compositional" method, this means the 
// Hessian approximation need only be computed
//once, and remains constant for all iterations(保持不变).

  float alpha = 1e-4;
  H <<
  h00+alpha ,h01      ,h02      ,h03,
  h10       ,h11+alpha,h12      ,h13,
  h20       ,h21      ,h22+alpha,h23,
  h30       ,h31      ,h32      ,h33+alpha;

  W = transformation;

  // Iterate
  int iter=0; // number of current iteration
  while(iter < max_iterations)
  {
    target.copyTo(debug);
    iter++; // Increment iteration counter

    uint pixel_count = 0; // Count of processed pixels 成功匹配的像素数
    float b0=0.0, b1=0.0, b2=0.0, b3=0.0; // 构建线性方程右侧向量b的累加器
    float mean_error = 0.0; // 平均像素误差，反映当前变换的匹配程度
        
    #pragma omp parallel for \
    reduction(+:mean_error, pixel_count, b0, b1, b2, b3)
    for(int row=0; row<source.rows; row++)
    {
      #pragma unroll
      for(int col=0; col<source.cols; col++)
      {
        // Set vector X with pixel coordinates (u,v,1)
        // 像素在 source 中的坐标
        X = Eigen::Vector3f(row, col, 1.0);
        Z = W*X; // 经过当前变换 W 后，在 target 中的坐标
        
        float row2 = Z(0);
        float col2 = Z(1);

        // Get the nearest integer pixel coords (u2i;v2i).
        int row2i = int(floor(row2));
        int col2i = int(floor(col2));

        if(row2i>=0 && row2i<target.rows && // check if pixel is inside I.
          col2i>=0 && col2i<target.cols)
        {
          pixel_count++;

          // Calculate intensity of a transformed pixel with sub-pixel accuracy
          // using bilinear interpolation.
          // 使用双线性插值获取目标图像上 (row2, col2) 处的灰度值（因为映射后的坐标可能是浮点数）
          float I2 = Interpolate<float>(target, row2, col2);
          
          debug.at<float>(row2i,col2i) = source.at<float>(row,col);

          // Calculate image difference D = I(W(x,p))-T(x).
          // 计算源图像和目标图像之间的像素差值（残差）
          float D =  source.at<float>(row, col) -I2;

          // Update mean error value.
          mean_error += fabsf(D);

          // Add a term to b matrix.

          Eigen::Vector4f db;
          db << steepest_descent.at<cv::Vec4f>(row, col)[0],
                steepest_descent.at<cv::Vec4f>(row, col)[1],
                steepest_descent.at<cv::Vec4f>(row, col)[2],
                steepest_descent.at<cv::Vec4f>(row, col)[3];
         
          // 对残差使用 Huber 损失函数进行加权，抑制异常值的影响
          db *= (fabsf(D) < HUBER_LOSS) ? D : D*HUBER_LOSS/fabsf(D);

          if(!std::isnan(db.dot(db))) 
          {
            b0 += db(0); 
            b1 += db(1); 
            b2 += db(2); 
            b3 += db(3); 

          }
        } 
      }
    }
    std::cout<< "residual:" << mean_error/pixel_count << "\n"; 

    cv::imshow("Debug", debug);
    cv::waitKey(24);
    
    b = Eigen::Vector4f(b0,b1,b2,b3);

    // Lucas-Kanade 20 Years On: A Unifying Framework 对应式子(51) G-N 增量方程
    // 利用之前计算好的 Hessian 矩阵 H 和当前的 b 向量，求解参数更新量 delta_p
    delta_p = H.ldlt().solve(b);

    Eigen::Matrix2f skew;
    skew << 0.0, -1.0, 
            1.0,  0.0;

    // Rodrigues' formula:
    // delta_p = [dx,dy,rotation,scale]
    // 这里旋转轴为 [0,0,1]
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() + sinf(delta_p(2))*skew + 
      (1-cosf(delta_p(2)))*(skew*skew.transpose() - Eigen::Matrix2f::Identity());
    
    // 构造包含平移、旋转、缩放的 3x3 变换矩阵 [dx,dy,rotation,scale]
    Eigen::Matrix3f T;
    T << R(0,0)+delta_p(3), R(0,1),             delta_p(0),
         R(1,0),            R(1,1)+delta_p(3),  delta_p(1),
            0.0,                          0.0,          1;
 
    dW = T*W;
    W = dW; // update W

    // Check termination critera.
    if(delta_p.norm()<=EPS)
      {      
        transformation = W;
        std::cout << "Terminated in " << iter << " iterations." <<std::endl;
        std::cout<<"transformation:\n"<<transformation<<std::endl;
        return;
      }
  } // iteration
  std::cout << "Maximum iterations reached (" << iter << ")." <<std::endl;
  transformation = W;
  std::cout<<"transformation:\n"<<transformation<<std::endl;
  return;

  // clang-format on
}

int main(int argc, char **argv) {
  // image data copy for github.com:dcanelhas/sim2-alignment.git
  cv::Mat img_src_c =
      cv::imread("./data/inverse_lk_flow/cat_src.png"); // 加了仿射变换后的图
  cv::Mat img_trg_c = cv::imread("./data/inverse_lk_flow/cat.png"); // 原图

  // 为仿射变换提供x y轴偏移的初始猜测

  // lena_src.png lena.png 150 50
  // cameraman_src.png cameraman.png
  // cat_src.png cat.png 160 160

  float x_offset = 160;
  float y_offset = 160;

  // convert images to float representation 图像转为浮点数表示
  cv::Mat img_src_f;
  cv::Mat img_trg_f;
  img_src_c.convertTo(img_src_f, CV_32F);
  img_trg_c.convertTo(img_trg_f, CV_32F);

  // rescale to unit 归一化到255
  img_src_f /= 255;
  img_trg_f /= 255;

  // containers for low-pass filtered imates
  cv::Mat img_src_blur;
  cv::Mat img_trg_blur;

  // Create a scale-space pyramid(创建比例空间金字塔) by low-pass filtering and
  // downsampling all the way to quarter-size images(四分之一图像)

  double sigma =
      0.95; // computed from filter size n=3 in [sigma = 0.3(n/2 - 1) + 0.8]
  cv::GaussianBlur(img_src_f, img_src_blur, cv::Size(3, 3), sigma);
  cv::GaussianBlur(img_trg_f, img_trg_blur, cv::Size(3, 3), sigma);
  cv::Mat img_src_half;
  cv::Mat img_trg_half;
  cv::resize(img_src_blur, img_src_half, cv::Size(0, 0), 0.5, 0.5);
  cv::resize(img_trg_blur, img_trg_half, cv::Size(0, 0), 0.5, 0.5);

  cv::GaussianBlur(img_src_half, img_src_blur, cv::Size(3, 3), sigma);
  cv::GaussianBlur(img_trg_half, img_trg_blur, cv::Size(3, 3), sigma);
  cv::Mat img_src_quarter;
  cv::Mat img_trg_quarter;
  cv::resize(img_src_blur, img_src_quarter, cv::Size(0, 0), 0.5, 0.5);
  cv::resize(img_trg_blur, img_trg_quarter, cv::Size(0, 0), 0.5, 0.5);

  cv::GaussianBlur(img_src_quarter, img_src_blur, cv::Size(3, 3), sigma);
  cv::GaussianBlur(img_trg_quarter, img_trg_blur, cv::Size(3, 3), sigma);
  cv::Mat img_src_eigth;
  cv::Mat img_trg_eigth;
  cv::resize(img_src_blur, img_src_eigth, cv::Size(0, 0), 0.5, 0.5);
  cv::resize(img_trg_blur, img_trg_eigth, cv::Size(0, 0), 0.5, 0.5);

  // offset 初始的仿射变换猜测
  Eigen::Matrix3f initial_guess;

  // 缩放默认为1/8 金字塔光流从粗到细
  initial_guess << 1.0, 0.0, x_offset / 8, 0.0, 1.0, y_offset / 8, 0, 0, 1;
  std::cout << "W:" << std::endl;
  std::cout << initial_guess << std::endl;

  Align(img_src_eigth, img_trg_eigth, 160, initial_guess);

#if PAUSE_BETWEEN
  cv::waitKey();
#endif

  initial_guess(0, 2) *= 2;
  initial_guess(1, 2) *= 2;
  Align(img_src_quarter, img_trg_quarter, 80, initial_guess);

#if PAUSE_BETWEEN
  cv::waitKey();
#endif

  initial_guess(0, 2) *= 2;
  initial_guess(1, 2) *= 2;
  Align(img_src_half, img_trg_half, 40, initial_guess);

#if PAUSE_BETWEEN
  cv::waitKey();
#endif

  initial_guess(0, 2) *= 2;
  initial_guess(1, 2) *= 2;
  Align(img_src_f, img_trg_f, 20, initial_guess);

  cv::waitKey();
  std::cout << initial_guess << std::endl;

  return 0;
}
