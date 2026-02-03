//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller.h"

namespace rm_chassis_controllers
{
Recover::Recover(const std::vector<hardware_interface::JointHandle*>& joint_handles,
                 const std::vector<control_toolbox::Pid*>& pid_legs,
                 const std::vector<control_toolbox::Pid*>& pid_thetas)
  : joint_handles_(joint_handles), pid_legs_(pid_legs), pid_thetas_(pid_thetas)
{
}

void Recover::execute(BipedalController* controller, const ros::Time& time, const ros::Duration& period)
{
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter RECOVER");
    detectLegRelState(left_pos_[1], left_leg_state);
    detectLegRelState(right_pos_[1], right_leg_state);
    controller->setStateChange(true);
  }

  int left_leg_state, right_leg_state;
  double left_desired_angle{ left_pos_[1] }, right_desired_angle{ right_pos_[1] };
  left_desired_angle = left_pos_[1];
  right_desired_angle = right_pos_[1];
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  setUpLegMotion(x_left_, right_leg_state, left_pos_[0], left_pos_[1], left_leg_state, left_desired_angle);
  setUpLegMotion(x_right_, left_leg_state, right_pos_[0], right_pos_[1], right_leg_state, right_desired_angle);
  if (controller->getBaseState() != 4)
  {
    left_cmd =
        computePidLegCommand(0.36, left_desired_angle, left_pos_, left_spd_, *pid_legs_[0], *pid_thetas_[0],
                             *pid_thetas_[2], left_angle_, left_leg_state, period, 0.0, controller->getOverturn());
    right_cmd =
        computePidLegCommand(0.36, right_desired_angle, right_pos_, right_spd_, *pid_legs_[1], *pid_thetas_[1],
                             *pid_thetas_[3], right_angle_, right_leg_state, period, 0.0, controller->getOverturn());
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd);

  // Exit
  if (!controller->getOverturn() && abs(x_left_[5]) < 0.2 && abs(x_left_[1]) < 0.2 && abs(x_right_[1]) < 0.2)
  {
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit RECOVER");
  }
}

void Recover::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const int& other_leg_state,
                             const double& leg_length, const double& leg_theta, int& leg_state, double& theta_des)
{
  switch (leg_state)
  {
    case LegState::UNDER:
      theta_des = leg_theta;
      if (leg_length > 0.35)
      {
        leg_state = LegState::BEHIND;
      }
      break;
    case LegState::FRONT:
      theta_des = leg_theta;
      if (other_leg_state == LegState::FRONT)
      {
        theta_des = M_PI / 2 - 0.35;
      }
      break;
    case LegState::BEHIND:
      theta_des = leg_theta;
      if ((leg_theta < -M_PI / 2 + 0.7 && leg_theta > -M_PI) || (leg_theta < M_PI && leg_theta > M_PI - 0.5))
      {
        leg_state = LegState::FRONT;
      }
      break;
  }
}

void Recover::detectLegRelState(const double& leg_theta, int& leg_state)
{
  if (leg_theta > -M_PI / 2 + 0.7 && leg_theta < (M_PI / 2 - 1.4))
    leg_state = LegState::UNDER;
  else if ((leg_theta < -M_PI / 2 + 0.7 && leg_theta > -M_PI) || (leg_theta < M_PI && leg_theta > M_PI - 0.5))
    leg_state = LegState::FRONT;
  else if (leg_theta > (M_PI / 2 - 1.4) && leg_theta < M_PI - 0.5)
    leg_state = LegState::BEHIND;
  switch (leg_state)
  {
    case LegState::UNDER:
      ROS_INFO("[balance] leg_theta: %.3f Leg state: UNDER", leg_theta);
      break;
    case LegState::FRONT:
      ROS_INFO("[balance] leg_theta: %.3f Leg state: FRONT", leg_theta);
      break;
    case LegState::BEHIND:
      ROS_INFO("[balance] leg_theta: %.3f Leg state: BEHIND", leg_theta);
      break;
  }
}
}  // namespace rm_chassis_controllers
