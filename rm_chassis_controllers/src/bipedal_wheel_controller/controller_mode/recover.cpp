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
    controller->setStateChange(true);
  }

  int left_leg_state, right_leg_state;
  double left_desired_angle{ left_pos_[1] }, right_desired_angle{ right_pos_[1] };
  left_desired_angle = left_pos_[1];
  right_desired_angle = right_pos_[1];
  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  detectLegState(x_left_, left_leg_state);
  detectLegState(x_right_, right_leg_state);
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
  if (!controller->getOverturn() && abs(x_left_[4]) < 0.2 && abs(roll_) < 0.1)
  {
    controller->setMode(BalanceMode::STAND_UP);
    controller->setStateChange(false);
    ROS_INFO("[balance] Exit RECOVER");
  }
}
}  // namespace rm_chassis_controllers
