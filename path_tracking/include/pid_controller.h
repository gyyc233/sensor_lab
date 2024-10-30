#ifndef PATH_TRACKING_PID_CONTROLLER_H
#define PATH_TRACKING_PID_CONTROLLER_H

/**
 * 位置式PID实现
 */
class PID_controller {
private:
  double kp, ki, kd;
  double target; // 目标
  double upper, lower;
  double error = 0.0;     // 当前误差
  double pre_error = 0.0; // 未来误差
  double sum_error = 0.0; // 累积误差

public:
  PID_controller(double kp, double ki, double kd, double target, double upper,
                 double lower);

  /// @brief 设置目标
  /// @param target
  void setTarget(double target);

  /// @brief set PID gain
  /// @param kp
  /// @param ki
  /// @param kd
  void setK(double kp, double ki, double kd);

  /// @brief 设置边界
  /// @param upper
  /// @param lower
  void setBound(double upper, double lower);

  double calOutput(double state);

  void reset();

  /// @brief 设置累计误差
  /// @param sum_error
  void setSumError(double sum_error);
};

#endif // PATH_TRACKING_PID_CONTROLLER_H