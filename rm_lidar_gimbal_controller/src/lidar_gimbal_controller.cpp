#include "lidar_gimbal_controller.h"
#include <pluginlib/class_list_macros.h>

namespace rm_lidar_gimbal_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
}

}  // namespace rm_lidar_gimbal_controller

PLUGINLIB_EXPORT_CLASS(rm_lidar_gimbal_controller::Controller, controller_interface::ControllerBase)
