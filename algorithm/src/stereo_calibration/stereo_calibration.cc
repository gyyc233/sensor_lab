#include "stereo_calibration/stereo_calibration.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "eigen_type/eigen_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;
using namespace pcl;

namespace Algorithm {

void StereoCalib::initFileList(string dir, int first, int last) {
  fileList.clear();
  for (int cur = first; cur <= last; cur++) {
    string str_file = dir + "/" + to_string(cur) + ".jpg";
    fileList.push_back(str_file);
  }
  std::cout << "image files num: " << fileList.size() << std::endl;
}

void StereoCalib::cvMatToPcl(cv::Mat &mat) {
  const double max_z = 1.0e4;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int ki = 0; ki < mat.rows; ki++) {
    for (int kj = 0; kj < mat.cols; kj++) {
      pcl::PointXYZ pointXYZ;
      cv::Vec3f point = mat.at<cv::Vec3f>(ki, kj);
      if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
        continue;

      pointXYZ.x = point[0];
      pointXYZ.y = point[1];
      pointXYZ.z = point[2];

      if (pointXYZ.z <= 0)
        continue;
      cloud->points.push_back(pointXYZ);
    }
  }
  int num_points = cloud->points.size();
  cloud->height = 1;
  cloud->width = num_points;
  pcl::io::savePLYFileASCII("./stereo.ply", *cloud);
  return;
}

void StereoCalib::saveXYZ(string filename, const Mat &mat) {
  const double max_z = 1.0e4;
  ofstream fp(filename);
  if (!fp.is_open()) {
    std::cout << "打开点云文件失败" << endl;
    fp.close();
    return;
  }
  //遍历写入
  for (int y = 0; y < mat.rows; y++) {
    for (int x = 0; x < mat.cols; x++) {
      cv::Vec3f point = mat.at<cv::Vec3f>(y, x); //三通道浮点型
      if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
        continue;
      fp << point[0] << " " << point[1] << " " << point[2] << endl;
    }
  }
  fp.close();
}

void StereoCalib::saveDisp(const string filename, const Mat &mat) {
  ofstream fp(filename, ios::out);
  fp << mat.rows << endl;
  fp << mat.cols << endl;
  for (int y = 0; y < mat.rows; y++) {
    for (int x = 0; x < mat.cols; x++) {
      double disp = mat.at<short>(
          y, x); // 这里视差矩阵是CV_16S 格式的，故用 short 类型读取
      fp << disp << endl; // 若视差矩阵是 CV_32F 格式，则用 float 类型读取
    }
  }
  fp.close();
}

void StereoCalib::F_Gray2Color(Mat gray_mat, Mat &color_mat) {
  color_mat = Mat::zeros(gray_mat.size(), CV_8UC3);
  int rows = color_mat.rows, cols = color_mat.cols;

  Mat red = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
  Mat green = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
  Mat blue = Mat(gray_mat.rows, gray_mat.cols, CV_8U);
  Mat mask = Mat(gray_mat.rows, gray_mat.cols, CV_8U);

  subtract(gray_mat, Scalar(255), blue); // blue(I) = 255 - gray(I)
  red = gray_mat.clone();                // red(I) = gray(I)
  green = gray_mat.clone();              // green(I) = gray(I),if gray(I) < 128

  compare(green, 128, mask,
          CMP_GE); // green(I) = 255 - gray(I), if gray(I) >= 128
  subtract(green, Scalar(255), green, mask);
  convertScaleAbs(green, green, 2.0, 2.0);

  vector<Mat> vec;
  vec.push_back(red);
  vec.push_back(green);
  vec.push_back(blue);
  cv::merge(vec, color_mat);
}

Mat StereoCalib::F_mergeImg(Mat img1, Mat disp8) {
  Mat color_mat = Mat::zeros(img1.size(), CV_8UC3);

  Mat red = img1.clone();
  Mat green = disp8.clone();
  Mat blue = Mat::zeros(img1.size(), CV_8UC1);

  vector<Mat> vec;
  vec.push_back(red);
  vec.push_back(blue);
  vec.push_back(green);
  cv::merge(vec, color_mat);

  return color_mat;
}

int StereoCalib::stereoCalibrate(string intrinsic_filename,
                                 string extrinsic_filename) {
  img_size = cv::Size(640, 480);
  pat_size = cv::Size(7, 6); //每张棋盘寻找的角点个数是7*6个
  vector<int> idx;
  //左侧相机的角点坐标和右侧相机的角点坐标
  vector<vector<Point2f>> imagePoints[2];

  for (uint i = 0; i < fileList.size(); ++i) {
    vector<Point2f> leftPts, rightPts; // 存储左右相机的角点位置
    Mat rawImg = imread(fileList[i]);  //原始图像
    if (rawImg.empty()) {
      std::cout << "the Image is empty..." << fileList[i] << endl;
      continue;
    }
    //截取左右相机图片
    Rect leftRect(0, 0, img_size.width, img_size.height);
    Rect rightRect(img_size.width, 0, img_size.width, img_size.height);

    Mat leftRawImg = rawImg(leftRect);   //切分得到的左原始图像
    Mat rightRawImg = rawImg(rightRect); //切分得到的右原始图像

    Mat leftImg, rightImg, leftSimg, rightSimg, leftCimg, rightCimg, leftMask,
        rightMask;
    // BGT -> GRAY
    if (leftRawImg.type() == CV_8UC3)
      cvtColor(leftRawImg, leftImg, COLOR_BGR2GRAY); //转为灰度图
    else
      leftImg = leftRawImg.clone();
    if (rightRawImg.type() == CV_8UC3)
      cvtColor(rightRawImg, rightImg, COLOR_BGR2GRAY);
    else
      rightImg = rightRawImg.clone();

    img_size = leftImg.size();

    leftCimg = leftImg.clone();
    rightCimg = rightImg.clone();

    //寻找棋盘角点
    bool leftFound = findChessboardCorners(leftImg, pat_size, leftPts,
                                           cv::CALIB_CB_ADAPTIVE_THRESH |
                                               cv::CALIB_CB_FILTER_QUADS);
    bool rightFound = findChessboardCorners(rightImg, pat_size, rightPts,
                                            cv::CALIB_CB_ADAPTIVE_THRESH |
                                                cv::CALIB_CB_FILTER_QUADS);

    if (!(leftFound && rightFound))
      continue;

    if (leftFound)
      cornerSubPix(
          leftImg, leftPts, Size(11, 11), Size(-1, -1),
          TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.01));
    if (rightFound)
      cornerSubPix(rightImg, rightPts, Size(11, 11), Size(-1, -1),
                   TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300,
                                0.01)); //亚像素

    //放大为原来的尺度
    for (uint j = 0; j < leftPts.size(); j++)
      leftPts[j] *= 1. / imgScale;
    for (uint j = 0; j < rightPts.size(); j++)
      rightPts[j] *= 1. / imgScale;

    //显示
    string leftWindowName = "Left Corner Pic",
           rightWindowName = "Right Corner Pic";

    Mat leftPtsTmp = Mat(leftPts) * imgScale; //再次乘以 imgScale
    Mat rightPtsTmp = Mat(rightPts) * imgScale;

    drawChessboardCorners(leftCimg, pat_size, leftPtsTmp,
                          leftFound); //绘制角点坐标并显示
    imshow(leftWindowName, leftCimg);
    imwrite("./lll_" + to_string(i) + "_left.jpg", leftCimg);
    waitKey(200);

    drawChessboardCorners(rightCimg, pat_size, rightPtsTmp,
                          rightFound); //绘制角点坐标并显示
    imshow(rightWindowName, rightCimg);
    imwrite("./rrr" + to_string(i) + "_right.jpg", rightCimg);
    waitKey(200);

    cv::destroyAllWindows();

    //保存角点坐标
    if (leftFound && rightFound) {
      imagePoints[0].push_back(leftPts);
      imagePoints[1].push_back(rightPts); //保存角点坐标
      std::cout << "图片 " << i << " 处理成功！" << endl;
      idx.push_back(i);
    }
  }
  cv::destroyAllWindows();
  imagePoints[0].resize(idx.size());
  imagePoints[1].resize(idx.size());
  std::cout << "成功标定的标定板个数为" << idx.size() << "  序号分别为: ";
  for (unsigned int i = 0; i < idx.size(); ++i)
    std::cout << idx[i] << "  ";

  //生成物点坐标
  vector<vector<Point3f>> objPts(
      idx.size()); // idx.size代表成功检测的图像的个数
  for (int y = 0; y < pat_size.height; y++) {
    for (int x = 0; x < pat_size.width; x++) {
      objPts[0].push_back(Point3f((float)x, (float)y, 0) * patLen);
    }
  }
  for (uint i = 1; i < objPts.size(); i++) {
    objPts[i] = objPts[0];
  }

  //
  // 双目立体标定
  Mat cameraMatrix[2], distCoeffs[2];
  vector<Mat> rvecs[2], tvecs[2];
  cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
  cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
  Mat R, T, E, F;

  cv::calibrateCamera(objPts, imagePoints[0], img_size, cameraMatrix[0],
                      distCoeffs[0], rvecs[0], tvecs[0]);

  cv::calibrateCamera(objPts, imagePoints[1], img_size, cameraMatrix[1],
                      distCoeffs[1], rvecs[1], tvecs[1]);

  std::cout << endl
            << "Left Camera Matrix: " << endl
            << cameraMatrix[0] << endl;
  std::cout << endl
            << "Right Camera Matrix: " << endl
            << cameraMatrix[1] << endl;
  std::cout << endl
            << "Left Camera DistCoeffs: " << endl
            << distCoeffs[0] << endl;
  std::cout << endl
            << "Right Camera DistCoeffs: " << endl
            << distCoeffs[1] << endl;

  double rms = cv::stereoCalibrate(
      objPts, imagePoints[0], imagePoints[1], cameraMatrix[0], distCoeffs[0],
      cameraMatrix[1], distCoeffs[1], img_size, R, T, E, F);
  // CV_CALIB_USE_INTRINSIC_GUESS);

  std::cout << endl
            << endl
            << "立体标定完成！ " << endl
            << "done with RMS error=" << rms << endl; //反向投影误差
  std::cout << endl
            << "Left Camera Matrix: " << endl
            << cameraMatrix[0] << endl;
  std::cout << endl
            << "Right Camera Matrix: " << endl
            << cameraMatrix[1] << endl;
  std::cout << endl
            << "Left Camera DistCoeffs: " << endl
            << distCoeffs[0] << endl;
  std::cout << endl
            << "Right Camera DistCoeffs: " << endl
            << distCoeffs[1] << endl;

  // 标定精度检测
  // 通过检查图像上点与另一幅图像的极线的距离来评价标定的精度。为了实现这个目的，使用
  // undistortPoints 来对原始点做去畸变的处理 随后使用 computeCorrespondEpilines
  // 来计算极线，计算点和线的点积。累计的绝对误差形成了误差
  std::cout << endl << " 极线计算...  误差计算... ";
  double err = 0;
  int npoints = 0;
  vector<cv::Vec3f> lines[2];
  for (unsigned int i = 0; i < idx.size(); i++) {
    int npt = (int)imagePoints[0][i].size(); //角点个数
    Mat imgpt[2];
    for (int k = 0; k < 2; k++) {
      imgpt[k] = Mat(imagePoints[k][i]); //
      undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(),
                      cameraMatrix[k]);                        // 畸变
      computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]); // 计算极线
    }
    for (int j = 0; j < npt; j++) {
      double errij =
          fabs(imagePoints[0][i][j].x * lines[1][j][0] +
               imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
          fabs(imagePoints[1][i][j].x * lines[0][j][0] +
               imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
      err += errij; // 累计误差
    }
    npoints += npt;
  }
  std::cout << "  平均误差 average reprojection err = " << err / npoints
            << endl; // 平均误差

  // 相机内参数和畸变系数写入文件
  FileStorage fs(intrinsic_filename, FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2"
       << cameraMatrix[1] << "D2" << distCoeffs[1];
    fs.release();
  } else
    std::cout << "Error: can not save the intrinsic parameters\n";

  // 立体矫正  BOUGUET'S METHOD
  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];
  // 立体矫正函数
  // R1 第一个相机的矫正变换矩阵（旋转）P1 第一个摄像机在新坐标系下的投影矩阵
  // Q 深度差异映射矩阵
  cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                    distCoeffs[1], img_size, R, T, R1, R2, P1, P2, Q,
                    CALIB_ZERO_DISPARITY, 1, img_size, &validRoi[0],
                    &validRoi[1]);

  fs.open(extrinsic_filename, FileStorage::WRITE);
  if (fs.isOpened()) {
    fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2"
       << P2 << "Q" << Q;
    fs.release();
  } else
    std::cout << "Error: can not save the intrinsic parameters\n";

  std::cout << "双目标定完成..." << endl;
  // getchar();getchar();
  return 0;
}

int StereoCalib::stereoMatch(int picNum, string intrinsic_filename,
                             string extrinsic_filename, bool no_display,
                             string point_cloud_filename) {
  //获取待处理的左右相机图像
  int color_mode = 0;
  Mat rawImg = imread(fileList[picNum], color_mode); //待处理图像  grayScale
  if (rawImg.empty()) {
    std::cout << "In Function stereoMatch, the Image is empty..." << endl;
    return 0;
  }
  //截取
  Rect leftRect(0, 0, img_size.width, img_size.height);
  Rect rightRect(img_size.width, 0, img_size.width, img_size.height);
  Mat img1 = rawImg(leftRect);  //切分得到的左原始图像
  Mat img2 = rawImg(rightRect); //切分得到的右原始图像
                                //图像根据比例缩放
  if (imgScale != 1.f) {
    Mat temp1, temp2;
    int method = imgScale < 1 ? INTER_AREA : INTER_CUBIC;
    resize(img1, temp1, Size(), imgScale, imgScale, method);
    img1 = temp1;
    resize(img2, temp2, Size(), imgScale, imgScale, method);
    img2 = temp2;
  }
  imwrite("./stereo_Match_origin_left.jpg", img1);
  imwrite("./stereo_Match_origin_right.jpg", img2);

  Size img_size = img1.size();

  // reading intrinsic parameters
  FileStorage fs(intrinsic_filename, FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to open file " << intrinsic_filename << endl;
    return -1;
  }
  Mat M1, D1, M2, D2; //左右相机的内参数矩阵和畸变系数
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  M1 *= imgScale;
  M2 *= imgScale;

  // 读取双目相机的立体矫正参数
  fs.open(extrinsic_filename, FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to open file  " << extrinsic_filename << endl;
    return -1;
  }

  // 立体矫正
  Rect roi1, roi2;
  Mat Q;
  Mat R, T, R1, P1, R2, P2;
  fs["R"] >> R;
  fs["T"] >> T;

  // Alpha取值为-1时，OpenCV自动进行缩放和平移
  cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q,
                    CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

  // 获取两相机的矫正映射
  Mat map11, map12, map21, map22;
  initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

  // 矫正原始图像
  Mat img1r, img2r;
  remap(img1, img1r, map11, map12, INTER_LINEAR);
  remap(img2, img2r, map21, map22, INTER_LINEAR);
  img1 = img1r;
  img2 = img2r;

  // 初始化 stereoBMstate 结构体
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);

  // bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
  bm->setPreFilterSize(9);
  bm->setPreFilterCap(31);
  bm->setBlockSize(21);
  bm->setMinDisparity(-16);
  bm->setNumDisparities(64);
  bm->setTextureThreshold(10);
  bm->setUniquenessRatio(5);
  bm->setSpeckleWindowSize(100);
  bm->setSpeckleRange(32);
  bm->setROI1(roi1);
  bm->setROI2(roi2);

  // int unitDisparity = 15; // 40
  // int numberOfDisparities = unitDisparity * 16;
  // bm.state->roi1 = roi1;
  // bm.state->roi2 = roi2;
  // bm.state->preFilterCap = 13;
  // bm.state->SADWindowSize = 19; // 窗口大小
  // bm.state->minDisparity = 0; // 确定匹配搜索从哪里开始  默认值是0
  // bm.state->numberOfDisparities =
  //     numberOfDisparities; // 在该数值确定的视差范围内进行搜索
  // bm.state->textureThreshold =
  //     1000; // 10                                  //
  //     保证有足够的纹理以克服噪声
  // bm.state->uniquenessRatio =
  //     1; // 10                               // !!使用匹配功能模式
  // bm.state->speckleWindowSize =
  //     200; // 13                             //
  //          // 检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查
  // bm.state->speckleRange =
  //     32; // 32                                  //
  //         //
  //         视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int
  //         // 型
  // bm.state->disp12MaxDiff = -1;

  // 计算
  Mat disp, disp8;
  int64 t = getTickCount();
  bm->compute(img1, img2, disp);
  t = getTickCount() - t;
  printf("立体匹配耗时: %fms\n", t * 1000 / getTickFrequency());

  // 将16位符号整形的视差矩阵转换为8位无符号整形矩阵
  disp.convertTo(disp8, CV_8U, 255 / (15 * 16.));

  // 视差图转为彩色图
  Mat vdispRGB = disp8;
  F_Gray2Color(disp8, vdispRGB);
  // 将左侧矫正图像与视差图融合
  Mat merge_mat = F_mergeImg(img1, disp8);

  saveDisp("输出/视差数据.txt", disp);

  //显示
  if (!no_display) {
    imshow("左侧矫正图像", img1);
    imwrite("输出/left_undistortRectify.jpg", img1);
    imshow("右侧矫正图像", img2);
    imwrite("输出/right_undistortRectify.jpg", img2);
    imshow("视差图", disp8);
    imwrite("输出/视差图.jpg", disp8);
    imshow("视差图_彩色.jpg", vdispRGB);
    imwrite("输出/视差图_彩色.jpg", vdispRGB);
    imshow("左矫正图像与视差图合并图像", merge_mat);
    imwrite("输出/左矫正图像与视差图合并图像.jpg", merge_mat);
    cv::waitKey();
    std::cout << endl;
  }
  cv::destroyAllWindows();

  // 视差图转为深度图
  cout << endl << "计算深度映射... " << endl;
  Mat xyz;
  reprojectImageTo3D(disp, xyz, Q, true); //获得深度图  disp: 720*1280
  cv::destroyAllWindows();
  cout << endl << "保存点云坐标... " << endl;
  saveXYZ(point_cloud_filename, xyz);
  cvMatToPcl(xyz);

  cout << endl << endl << "结束" << endl << "Press any key to end... ";

  getchar();
  return 0;
}

void StereoCalib::stereoDemo(string left_img_dir, string right_img_dir) {
  // 内参
  double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
  // 基线
  double b = 0.573;

  // 读取图像
  cv::Mat left = cv::imread(left_img_dir, 0);
  cv::Mat right = cv::imread(right_img_dir, 0);
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
      0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32); // 神奇的参数
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (int v = 0; v < left.rows; v++) {
    for (int u = 0; u < left.cols; u++) {
      if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
        continue;

      Eigen::Vector4d point(
          0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色
      pcl::PointXYZRGB pointXYZRGB;
      // 根据双目模型计算 point 的位置
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      double depth = fx * b / (disparity.at<float>(v, u));
      point[0] = x * depth;
      point[1] = y * depth;
      point[2] = depth;

      // 视差图不能直接用到点云上，至少要转为深度图，再转点云
      pointXYZRGB.x = static_cast<float>(point[0]);
      pointXYZRGB.y = static_cast<float>(point[1]);
      pointXYZRGB.z = static_cast<float>(point[2]);
      pointXYZRGB.r = static_cast<float>(point[3]);
      pointXYZRGB.g = static_cast<float>(point[3]);
      pointXYZRGB.b = static_cast<float>(point[3]);

      pointcloud.push_back(point);
      cloud->push_back(pointXYZRGB);
    }
  }

  int num_points = cloud->points.size();
  cloud->height = 1;
  cloud->width = num_points;
  pcl::io::savePLYFileASCII("./stereo_demo.ply", *cloud);

  cv::imshow("disparity", disparity / 96.0);
  cv::waitKey(0);
}

// string intrinsic_filename = "intrinsics.yml";
// string extrinsic_filename = "extrinsics.yml";
// string point_cloud_filename = "输出/point3D.txt";

// /* 立体标定 运行一次即可 */
// initFileList("calib_pic", 1, 26);
// stereoCalibrate(intrinsic_filename, extrinsic_filename);

// /* 立体匹配 */
// initFileList("test_pic", 1, 2);
// stereoMatch(0, intrinsic_filename, extrinsic_filename, false,
// point_cloud_filename);

} // namespace Algorithm
