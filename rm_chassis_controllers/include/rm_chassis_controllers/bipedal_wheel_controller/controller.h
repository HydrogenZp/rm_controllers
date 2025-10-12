//
// Created by guanlin on 25-8-28.
//

#pragma once

#include <rm_common/lqr.h>
#include "rm_msgs/LegCmd.h"
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "rm_chassis_controllers/chassis_base.h"

#include "bipedal_wheel_controller/helper_functions.h"
#include "bipedal_wheel_controller/definitions.h"
#include "bipedal_wheel_controller/controller_mode/mode_manager.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // clang-format off
  bool getOverturn() const{ return overturn_; }
  bool getStateChange() const{ return balance_state_changed_; }
  bool getCompleteStand() const{ return complete_stand_; }
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> getCoeffs() { return coeffs_; }
  const std::shared_ptr<ModelParams>& getModelParams(){ return model_params_; }
  double getLegCmd() const{ return legCmd_; }
  double getJumpCmd() const{ return jumpCmd_; }
  geometry_msgs::Vector3 getVelCmd(){ return vel_cmd_; }

  void setStateChange(bool state){ balance_state_changed_ = state; }
  void setCompleteStand(bool state){ complete_stand_ = state; }
  void setJumpCmd(bool cmd){ jumpCmd_ = cmd; }
  void setMode(int mode){ balance_mode_ = mode; }
  void pubState();
  // clang-format on

private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupLQR(ros::NodeHandle& controller_nh);
  bool setupBiasParams(ros::NodeHandle& controller_nh);
  void polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
               Eigen::Matrix<double, 4, 12>& coeffs);
  geometry_msgs::Twist odometry() override;
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> coeffs_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};

  std::shared_ptr<ModelParams> model_params_;
  std::shared_ptr<BiasParams> bias_params_;

  int balance_mode_ = BalanceMode::SIT_DOWN;
  bool balance_state_changed_ = false;
  std::unique_ptr<ModeManager> mode_manager_;

  // stand up
  bool complete_stand_ = false, overturn_ = false;

  // handles
  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_hip_joint_handle_,
      left_knee_joint_handle_, right_hip_joint_handle_, right_knee_joint_handle_;
  std::vector<hardware_interface::JointHandle*> joint_handles_;

  // Leg Cmd
  double legCmd_{};
  bool jumpCmd_{};

  // ROS Interface
  ros::Subscriber leg_cmd_sub_;
  ros::Publisher unstick_pub_;
  ros::Time cmd_update_time_;
};
}  // namespace rm_chassis_controllers
