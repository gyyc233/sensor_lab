#include "solve_5pts.h"

namespace cv {

// 将本质矩阵（Essential Matrix）分解为两个可能的旋转矩阵和一个平移向量
void decomposeEssentialMat(InputArray _E, OutputArray _R1, OutputArray _R2,
                           OutputArray _t) {

  Mat E = _E.getMat().reshape(1, 3);
  CV_Assert(E.cols == 3 && E.rows == 3);

  // 对E做SVD分解
  Mat D, U, Vt;
  SVD::compute(E, D, U, Vt);

  // 计算左右奇异矩阵的行列式是否为正，若为负数则取反，保证是合法的旋转矩阵
  if (determinant(U) < 0)
    U *= -1.;
  if (determinant(Vt) < 0)
    Vt *= -1.;

  //构造固定的反对称矩阵
  Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
  W.convertTo(W, E.type());

  // 计算两种旋转矩阵R1和R2
  Mat R1, R2, t;
  R1 = U * W * Vt;
  R2 = U * W.t() * Vt;
  // 表示归一化的平移方向
  t = U.col(2) * 1.0;

  R1.copyTo(_R1);
  R2.copyTo(_R2);
  t.copyTo(_t);
  // 本质矩阵只能提供方向信息，不能确定平移的尺度，因此后续通常需要结合三角化或重投影误差等方法来选择正确的解
}

// 从本质矩阵（Essential
// Matrix）中恢复出最优的相机相对旋转和平移变换，结合了特征点、相机内参等信息的几何验证
int recoverPose(InputArray E, InputArray _points1, InputArray _points2,
                InputArray _cameraMatrix, OutputArray _R, OutputArray _t,
                InputOutputArray _mask) {
  // 统一输入变量格式到CV_64F
  Mat points1, points2, cameraMatrix;
  _points1.getMat().convertTo(points1, CV_64F);
  _points2.getMat().convertTo(points2, CV_64F);
  _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

  int npoints = points1.checkVector(2);
  CV_Assert(npoints >= 0 && points2.checkVector(2) == npoints &&
            points1.type() == points2.type());

  CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 &&
            cameraMatrix.channels() == 1);

  // 如果点是以多通道形式存储（如 Vec2f），则将其重塑为单通道矩阵
  if (points1.channels() > 1) {
    points1 = points1.reshape(1, npoints);
    points2 = points2.reshape(1, npoints);
  }

  // 像素坐标准尉归一化相机坐标系下坐标（去除了焦距和平移影响）
  // 为了与本质矩阵定义一致（本质矩阵作用与归一化坐标）
  double fx = cameraMatrix.at<double>(0, 0);
  double fy = cameraMatrix.at<double>(1, 1);
  double cx = cameraMatrix.at<double>(0, 2);
  double cy = cameraMatrix.at<double>(1, 2);

  points1.col(0) = (points1.col(0) - cx) / fx;
  points2.col(0) = (points2.col(0) - cx) / fx;
  points1.col(1) = (points1.col(1) - cy) / fy;
  points2.col(1) = (points2.col(1) - cy) / fy;

  points1 = points1.t();
  points2 = points2.t();

  // 基于decomposeEssentialMat得到两个旋转和一个平移方向
  Mat R1, R2, t;
  decomposeEssentialMat(E, R1, R2, t);
  Mat P0 = Mat::eye(3, 4, R1.type());
  Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()),
      P4(3, 4, R1.type());

  // 构造四个可能的投影矩阵组合
  P1(Range::all(), Range(0, 3)) = R1 * 1.0;
  P1.col(3) = t * 1.0;

  P2(Range::all(), Range(0, 3)) = R2 * 1.0;
  P2.col(3) = t * 1.0;

  P3(Range::all(), Range(0, 3)) = R1 * 1.0;
  P3.col(3) = -t * 1.0;

  P4(Range::all(), Range(0, 3)) = R2 * 1.0;
  P4.col(3) = -t * 1.0;

  // Do the cheirality check.
  // Notice here a threshold dist is used to filter
  // out far away points (i.e. infinite points) since
  // there depth may vary between postive and negtive.
  double dist = 50.0; // 最大距离
  Mat Q;

  // 以下对四种运动假设分别进行三角化
  // 检查重建的三维点是否在相机前方
  triangulatePoints(P0, P1, points1, points2, Q);
  Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask1 = (Q.row(2) < dist) & mask1;
  Q = P1 * Q;
  mask1 = (Q.row(2) > 0) & mask1;
  mask1 = (Q.row(2) < dist) & mask1;

  triangulatePoints(P0, P2, points1, points2, Q);
  Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask2 = (Q.row(2) < dist) & mask2;
  Q = P2 * Q;
  mask2 = (Q.row(2) > 0) & mask2;
  mask2 = (Q.row(2) < dist) & mask2;

  triangulatePoints(P0, P3, points1, points2, Q);
  Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask3 = (Q.row(2) < dist) & mask3;
  Q = P3 * Q;
  mask3 = (Q.row(2) > 0) & mask3;
  mask3 = (Q.row(2) < dist) & mask3;

  triangulatePoints(P0, P4, points1, points2, Q);
  Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
  Q.row(0) /= Q.row(3);
  Q.row(1) /= Q.row(3);
  Q.row(2) /= Q.row(3);
  Q.row(3) /= Q.row(3);
  mask4 = (Q.row(2) < dist) & mask4;
  Q = P4 * Q;
  mask4 = (Q.row(2) > 0) & mask4;
  mask4 = (Q.row(2) < dist) & mask4;

  mask1 = mask1.t();
  mask2 = mask2.t();
  mask3 = mask3.t();
  mask4 = mask4.t();

  // If _mask is given, then use it to filter outliers.
  // 如果传入了初始掩码（比如来自 RANSAC 的掩码），则与当前掩码做交集
  if (!_mask.empty()) {
    Mat mask = _mask.getMat();
    CV_Assert(mask.size() == mask1.size());
    bitwise_and(mask, mask1, mask1);
    bitwise_and(mask, mask2, mask2);
    bitwise_and(mask, mask3, mask3);
    bitwise_and(mask, mask4, mask4);
  }
  if (_mask.empty() && _mask.needed()) {
    _mask.create(mask1.size(), CV_8U);
  }

  CV_Assert(_R.needed() && _t.needed());
  _R.create(3, 3, R1.type());
  _t.create(3, 1, t.type());

  int good1 = countNonZero(mask1);
  int good2 = countNonZero(mask2);
  int good3 = countNonZero(mask3);
  int good4 = countNonZero(mask4);

  // 选择内点最多的那一组作为最终结果，若选择了 -t，则将平移向量取反
  if (good1 >= good2 && good1 >= good3 && good1 >= good4) {
    R1.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed())
      mask1.copyTo(_mask);
    return good1;
  } else if (good2 >= good1 && good2 >= good3 && good2 >= good4) {
    R2.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed())
      mask2.copyTo(_mask);
    return good2;
  } else if (good3 >= good1 && good3 >= good2 && good3 >= good4) {
    t = -t;
    R1.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed())
      mask3.copyTo(_mask);
    return good3;
  } else {
    t = -t;
    R2.copyTo(_R);
    t.copyTo(_t);
    if (_mask.needed())
      mask4.copyTo(_mask);
    return good4;
  }
}

int recoverPose(InputArray E, InputArray _points1, InputArray _points2,
                OutputArray _R, OutputArray _t, double focal, Point2d pp,
                InputOutputArray _mask) {
  cv::Mat cameraMatrix =
      (Mat_<double>(3, 3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
  return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
}

} // namespace cv

namespace sensor_lab {

bool MotionEstimator::solveRelativeRT(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
    Eigen::Matrix3d &rotation, Eigen::Vector3d &translation) {
  if (corres.size() >= 15) {
    std::vector<cv::Point2f> ll, rr;
    for (int i = 0; i < int(corres.size()); i++) {
      ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
      rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
    }

    cv::Mat mask;
    // 基础矩阵，带有相机内参K,这里假设相机内参已归一化（单位矩阵）,所以这里基础矩阵等价与本质矩阵E
    cv::Mat E =
        cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    // 从Ｅ矩阵恢复RT
    // cv::recoverPose 返回的是从右帧到左帧的变换，因此需要转置旋转矩阵
    int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
    // cout << "inlier_cnt " << inlier_cnt << endl;

    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    // cv to eigen
    for (int i = 0; i < 3; i++) {
      T(i) = trans.at<double>(i, 0);
      for (int j = 0; j < 3; j++)
        R(i, j) = rot.at<double>(i, j);
    }

    rotation = R.transpose();
    translation = -R.transpose() *
                  T; // 将平移向量调整为 -R^T * T，以获得从左帧到右帧的相对变换
    if (inlier_cnt > 12)
      return true;
    else
      return false;
  }

  return false;
}

} // namespace sensor_lab
