#include "controller.h"

using namespace std;



double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}


///*************核心控制器****************//
/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
  // compute disired acceleration
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d des_v_out(0.0, 0.0, 0.0);

  static Eigen::Vector3d last_err_v(0.0, 0.0, 0.0);
  static Eigen::Vector3d err_v_i(0.0, 0.0, 0.0);

  Eigen::Vector3d kPp, kVp, kVi, kVd;
  kPp << param_.gain.kPp0, param_.gain.kPp1, param_.gain.kPp2;
  kVp << param_.gain.kVp0, param_.gain.kVp1, param_.gain.kVp2;
  kVi << param_.gain.kVi0, param_.gain.kVi1, param_.gain.kVi2;
  kVd << param_.gain.kVd0, param_.gain.kVd1, param_.gain.kVd2;

  des_v_out = kPp.asDiagonal() * (des.p - odom.p) + des.v;            // P
  des_acc = kVp.asDiagonal() * (des_v_out - odom.v)                   // V
          + kVi.asDiagonal() * err_v_i   //此处添加时间常数  没有实际意义 只能调整数位 数字离谱可以直接删去 对功能无影响
          + kVd.asDiagonal() * ((des_v_out - odom.v) - last_err_v)
          + des.a;

  des_acc += Eigen::Vector3d(0.0, 0.0, param_.gra);

  des_acc.x() = std::max(-param_.max_acc_xy, std::min(param_.max_acc_xy, des_acc.x())); // 加速度限制
  des_acc.y() = std::max(-param_.max_acc_xy, std::min(param_.max_acc_xy, des_acc.y()));

  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);

  double roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;  // a.y
  double pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra; // a.x

  // 姿态解算
  Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * 
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  u.q = imu.q * odom.q.inverse() * q;

  // PID变量赋值
  last_err_v = des_v_out - odom.v;

  if (!(fabs(des_acc.x()) >= param_.max_acc_xy - 1e-6))
      err_v_i.x() += des_v_out.x() - odom.v.x();
  if (!(fabs(des_acc.y()) >= param_.max_acc_xy - 1e-6))
      err_v_i.y() += des_v_out.y() - odom.v.y();
  if (!(fabs(des_acc.z()) >= param_.max_acc_xy - 1e-6))   // 不使用Z轴I系数 暂时做歧义编写
      err_v_i.z() += des_v_out.z() - odom.v.z();

  //used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des_v_out(0);
  debug_msg_.des_v_y = des_v_out(1);
  debug_msg_.des_v_z = des_v_out(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.err_p_x = des.p(0) - odom.p(0);
  debug_msg_.err_p_y = des.p(1) - odom.p(1);
  debug_msg_.err_p_z = des.p(2) - odom.p(2);

  debug_msg_.err_v_x = des_v_out(0) - odom.v(0);
  debug_msg_.err_v_y = des_v_out(1) - odom.v(1);
  debug_msg_.err_v_z = des_v_out(2) - odom.v(2);

  debug_msg_.err_q_x = q.x();
  debug_msg_.err_q_y = q.y();
  debug_msg_.err_q_z = q.z();
  debug_msg_.err_q_w = q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
