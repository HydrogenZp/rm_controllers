//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Normal::Normal(const std::vector<hardware_interface::JointHandle*>& joint_handles,
               const std::vector<control_toolbox::Pid*>& pid_legs, const control_toolbox::Pid& pid_yaw_pos,
               const control_toolbox::Pid& pid_yaw_vel, const control_toolbox::Pid& pid_theta_diff,
               const control_toolbox::Pid& pid_roll)
  : joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_yaw_pos_(pid_yaw_pos)
  , pid_yaw_vel_(pid_yaw_vel)
  , pid_theta_diff_(pid_theta_diff)
  , pid_roll_(pid_roll)
{
  supportForceAveragePtr = std::make_unique<MovingAverageFilter<double>>(4);
}

void Normal::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter NORMAL");
    yaw_total_ = yaw_total_last_ = 0.;
    yaw_des_ = yaw_total_;
    x_left_(2) = x_right_(2) = 0;
    controller->setStateChange(true);
  }
  if (!controller->getCompleteStand() && abs(x_left_[4]) < 0.2)
    controller->setCompleteStand(true);

  auto vel_cmd_ = controller->getVelCmd();

  // PID
  double T_yaw = pid_yaw_vel_.computeCommand(vel_cmd_.z - angular_vel_base_.z, period);
  double T_theta_diff = pid_theta_diff_.computeCommand(right_pos_[1] - left_pos_[1], period);
  double T_roll = pid_roll_.computeCommand(0. - roll_, period);

  // LQR
  Matrix<double, 4, 12> coeffs_ = controller->getCoeffs();
  Matrix<double, 2, 6> k_left, k_right;
  k_left.setZero();
  k_right.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      k_left(i, j) = coeffs_(0, i + 2 * j) * pow(left_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(left_pos_[0], 2) +
                     coeffs_(2, i + 2 * j) * left_pos_[0] + coeffs_(3, i + 2 * j);
      k_right(i, j) = coeffs_(0, i + 2 * j) * pow(right_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(right_pos_[0], 2) +
                      coeffs_(2, i + 2 * j) * right_pos_[0] + coeffs_(3, i + 2 * j);
    }
  }
  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  u_left.setZero();
  u_right.setZero();
  auto x_left = x_left_;
  auto x_right = x_right_;
  if (controller->getCompleteStand())
  {
    x_left(3) -= vel_cmd_.x;
    x_right(3) -= vel_cmd_.x;
  }
  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  controller->pubLQRStatus(-x_left, -x_right, u_left, u_right);

  // Compute leg thrust
  auto model_params_ = controller->getModelParams();
  double gravity = 1. / 2. * model_params_->M * model_params_->g;
  Eigen::Matrix<double, 2, 1> F_leg;
  double leg_length_des = controller->getLegCmd() == 0 ? 0.2 : controller->getLegCmd();
  if (!start_jump_ && controller->getJumpCmd() && abs(x_left[0]) < 0.1)
  {
    start_jump_ = true;
    ROS_INFO("[balance] Jump start");
  }
  if (start_jump_)
  {
    leg_length_des = jumpLengthDes[jump_phase_].second;
    if (std::abs(leg_length_des - left_pos_[0]) < 0.01)
      jump_phase_ += 1;
    if (jump_phase_ == JumpPhase::DONE)
    {
      jump_phase_ = JumpPhase::SQUAT;
      controller->setJumpCmd(false);
      start_jump_ = false;
      ROS_INFO("[balance] Jump finished");
    }
    else if (jump_phase_ == JumpPhase::SHRINK)
    {
      gravity = 0;
    }
    F_leg[0] =
        pid_legs_[0]->computeCommand(leg_length_des - left_pos_[0], period) + gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] =
        pid_legs_[1]->computeCommand(leg_length_des - right_pos_[0], period) + gravity * cos(right_pos_[1]) - T_roll;
  }
  else
  {
    double left_length_des = controller->getCompleteStand() ? leg_length_des / cos(x_left[0]) : 0.20;
    double right_length_des = controller->getCompleteStand() ? leg_length_des / cos(x_right[0]) : 0.20;
    F_leg[0] =
        pid_legs_[0]->computeCommand(left_length_des - left_pos_[0], period) + gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] =
        pid_legs_[1]->computeCommand(right_length_des - right_pos_[0], period) + gravity * cos(right_pos_[1]) - T_roll;
  }

  // Unstick detection
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick{}, k_right_unstick{};
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick.block<1, 2>(1, 0) = k_left.block<1, 2>(1, 0);
  k_right_unstick.block<1, 2>(1, 0) = k_right.block<1, 2>(1, 0);

  //  bool left_unstick =
  //      unstickDetection(joint_handles_[0]->getEffort(), joint_handles_[1]->getEffort(), joint_handles_[4]->getEffort(),
  //                       left_angle_[0], left_angle_[1], left_pos_[0], linear_acc_base_.z, model_params_, x_left_);
  //  bool right_unstick =
  //      unstickDetection(joint_handles_[2]->getEffort(), joint_handles_[3]->getEffort(), joint_handles_[5]->getEffort(),
  //                       right_angle_[0], right_angle_[1], right_pos_[0], linear_acc_base_.z, model_params_, x_right_);
  bool left_unstick = false;
  bool right_unstick = false;
  updateUnstick(left_unstick, right_unstick);

  if (left_unstick && jump_phase_ != JumpPhase::JUMP)
    u_left = k_left_unstick * (-x_left);
  if (right_unstick && jump_phase_ != JumpPhase::JUMP)
    u_right = k_right_unstick * (-x_right);

  // Control
  double left_T[2], right_T[2];
  leg_conv(F_leg[0], u_left(1) + T_theta_diff, left_angle_[0], left_angle_[1], left_T);
  leg_conv(F_leg[1], u_right(1) - T_theta_diff, right_angle_[0], right_angle_[1], right_T);
  double left_wheel_cmd = left_unstick ? 0. : u_left(0) - T_yaw;
  double right_wheel_cmd = right_unstick ? 0. : u_right(0) + T_yaw;
  LegCommand left_cmd = { F_leg[0], u_left[1], { left_T[0], left_T[1] } },
             right_cmd = { F_leg[1], u_right[1], { right_T[0], right_T[1] } };
  setJointCommands(joint_handles_, left_cmd, right_cmd, left_wheel_cmd, right_wheel_cmd);

  // Protection
  if ((controller->getCompleteStand() && (abs(x_left(4)) > 0.4 || abs(x_left(0)) > 1.5)) || controller->getOverturn() ||
      controller->getBaseState() == 4)
  {
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
    controller->setJumpCmd(false);
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    ROS_INFO("[balance] Exit NORMAL");
  }
}

double Normal::calculateSupportForce(double F, double Tp, double leg_length, double acc_z,
                                     Eigen::Matrix<double, STATE_DIM, 1> x, Eigen::Matrix<double, CONTROL_DIM, 1> u,
                                     const std::shared_ptr<ModelParams>& model_params)
{
  static double last_ddot_zM = acc_z - model_params->g;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a;
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b;
  generateAB(model_params, a, b, leg_length);

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  // lp filter
  double ddot_zM = 0.7 * (acc_z - model_params->g) + 0.3 * last_ddot_zM;
  auto ddot_x = a * x + b * u;
  double ddot_theta = ddot_x(1);
  double ddot_zw = ddot_zM - leg_length * cos(x(0)) + 2 * leg_length * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + x(1) * x(1) * cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;

  return Fn;
}

bool Normal::unstickDetection(const double& hip_effort, const double& knee_effort, const double& wheel_effort,
                              const double& hip_angle, const double& knee_angle, const double& leg_length,
                              const double& acc_z, const std::shared_ptr<ModelParams>& model_params,
                              Eigen::Matrix<double, STATE_DIM, 1> x)
{
  static bool maybeChange = false, last_unstick_ = false;
  ros::Time judgeTime;
  double leg_F[2];
  leg_conv_fwd(hip_effort, knee_effort, hip_angle, knee_angle, leg_F);
  Eigen::Matrix<double, CONTROL_DIM, 1> u_left_real;
  u_left_real << wheel_effort, leg_F[1];
  double Fn = calculateSupportForce(leg_F[0], leg_F[1], leg_length, acc_z, x, u_left_real, model_params);

  bool unstick_ = Fn < 20;

  if (unstick_ != last_unstick_)
  {
    if (!maybeChange)
    {
      judgeTime = ros::Time::now();
      maybeChange = true;
    }
    else
    {
      if (ros::Time::now() - judgeTime > ros::Duration(0.01))
      {
        last_unstick_ = unstick_;
      }
    }
  }
  else
  {
    maybeChange = false;
  }
  return unstick_;
}
}  // namespace rm_chassis_controllers
