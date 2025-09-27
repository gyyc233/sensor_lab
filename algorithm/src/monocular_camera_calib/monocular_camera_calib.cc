#include "monocular_camera_calib/monocular_camera_calib.h"
#include <dirent.h>
#include <iostream>
#include <sys/types.h>

namespace Algorithm {

void MonocularCameraCalib::SingleCalibrate(std::string intrinsic_filename,
                                           std::string pics_path) {
  //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
  int image_count = 0;                  /* 图像数量 */
  cv::Size image_size;                  /* 图像的尺寸 */
  cv::Size board_size = cv::Size(7, 6); /* 标定板上每行、列的角点数 */
  std::vector<cv::Point2f> image_points_buf; /* 缓存每幅图像上检测到的角点 */
  std::vector<std::vector<cv::Point2f>>
      image_points_seq; /* 保存检测到的所有角点 */
  std::string filename;
  int count = -1; //用于存储角点个数。
  //获得相机标定图像
  std::vector<std::string> fileList;
  InitFileList(pics_path, fileList);

  for (int i = 0; i < fileList.size(); ++i) {
    filename = fileList[i];
    image_count++;
    cv::Mat imageInput = cv::imread(filename);
    if (image_count == 1) //读入第一张图片时获取图像宽高信息
    {
      image_size.width = imageInput.cols;
      image_size.height = imageInput.rows;
      std::cout << "image_size.width = " << image_size.width << std::endl;
      std::cout << "image_size.height = " << image_size.height << std::endl;
    }

    if (0 == cv::findChessboardCorners(imageInput, board_size, image_points_buf,
                                       cv::CALIB_CB_ADAPTIVE_THRESH +
                                           cv::CALIB_CB_NORMALIZE_IMAGE +
                                           cv::CALIB_CB_FAST_CHECK)) {
      std::cout << "can not find chessboard corners!\n"; //找不到角点
      exit(1);
    } else {
      cv::Mat view_gray;
      cv::cvtColor(imageInput, view_gray, cv::COLOR_RGB2GRAY);
      cornerSubPix(
          view_gray, image_points_buf, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.1));
      image_points_seq.push_back(image_points_buf);
      cv::drawChessboardCorners(view_gray, board_size, image_points_buf, false);
      cv::imwrite("./output/SingleClib/point/" + std::to_string(i) + ".jpg",
                  view_gray);
      imshow("Camera Calibration", view_gray);
      cv::waitKey(500);
    }
  }

  int total = image_points_seq.size();
  std::cout << "image total = " << total << std::endl;

  std::cout << "single camera calibration";
  cv::Size square_size =
      cv::Size(10, 10); /* 实际测量得到的标定板上每个棋盘格的大小 */
  std::vector<std::vector<cv::Point3f>>
      object_points; /* 保存标定板上角点的三维坐标 */
  /*内外参数*/
  cv::Mat cameraMatrix =
      cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
  std::vector<int> point_counts; // 每幅图像中角点的数量
  cv::Mat distCoeffs =
      cv::Mat(1, 5, CV_32FC1,
              cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
  std::vector<cv::Mat> tvecsMat; /* 每幅图像的旋转向量 */
  std::vector<cv::Mat> rvecsMat; /* 每幅图像的平移向量 */
  /* 初始化标定板上角点的三维坐标 */
  int i, j, t;
  for (t = 0; t < image_count; t++) {
    std::vector<cv::Point3f> tempPointSet;
    for (i = 0; i < board_size.height; i++) {
      for (j = 0; j < board_size.width; j++) {
        cv::Point3f realPoint;
        /* 假设标定板放在世界坐标系中z=0的平面上 */
        realPoint.x = i * square_size.width;
        realPoint.y = j * square_size.height;
        realPoint.z = 0;
        tempPointSet.push_back(realPoint);
      }
    }
    object_points.push_back(tempPointSet);
  }
  /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
  for (i = 0; i < image_count; i++) {
    point_counts.push_back(board_size.width * board_size.height);
  }
  /* 开始标定 */
  cv::calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix,
                      distCoeffs, rvecsMat, tvecsMat, 0);
  std::cout << "finish camera calibration\n";
  std::cout << "cameraMatrix:\n" << cameraMatrix << std::endl;
  std::cout << "distCoeffs:\n" << distCoeffs << std::endl;
  //对标定结果进行评价
  std::cout << "eval calib result………………\n";
  double total_err = 0.0; /* 所有图像的平均误差的总和 */
  double err = 0.0;       /* 每幅图像的平均误差 */
  std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */

  for (i = 0; i < image_count; i++) {
    std::vector<cv::Point3f> tempPointSet = object_points[i];
    /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
     */
    cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix,
                      distCoeffs, image_points2);
    /* 计算新的投影点和旧的投影点之间的误差*/
    std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
    cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
    cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
    for (int j = 0; j < tempImagePoint.size(); j++) {
      image_points2Mat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(image_points2[j].x, image_points2[j].y);
      tempImagePointMat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
    }
    err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
    total_err += err /= point_counts[i];
  }
  std::cout << "总体平均误差：" << total_err / image_count << "像素"
            << std::endl;

  /************************************************************************
  显示定标结果
  *************************************************************************/
  cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
  cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  std::cout << "保存矫正图像" << std::endl;
  std::string imageFileName;
  std::stringstream StrStm;
  for (int i = 0; i != image_count; i++) {
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix,
                                image_size, CV_32FC1, mapx, mapy);
    StrStm.clear();

    cv::Mat imageSource = cv::imread(fileList[i]);
    cv::Mat newimage = imageSource.clone();
    //另一种不需要转换矩阵的方式
    // undistort(imageSource,newimage,cameraMatrix,distCoeffs);
    cv::remap(imageSource, newimage, mapx, mapy, cv::INTER_LINEAR);
    cv::imwrite("./left_remap_" + std::to_string(i) + ".jpg", newimage);
  }
  return;
}

void MonocularCameraCalib::InitFileList(std::string path,
                                        std::vector<std::string> &files) {
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(path.c_str())))
    return;
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
      files.push_back(path + "/" + ptr->d_name);
  }
  closedir(pDir);
}

}; // namespace Algorithm
