#include "lidar_gimbal_controller.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "rm_common/hardware_interface/robot_state_interface.h"

namespace rm_lidar_gimbal_controller
{
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  controller_nh.getParam("imu_name", imu_name_);
  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(imu_name_);

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (controller_nh.getParam("controllers", xml_rpc_value))
  {
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    for (const auto& it : xml_rpc_value)
    {
      const auto controller_name = it.first;
      ros::NodeHandle nh = ros::NodeHandle(controller_nh, "controllers/" + controller_name);
      ros::NodeHandle nh_pid_pos = ros::NodeHandle(controller_nh, "controllers/" + controller_name + "/pid_pos");

      std::string joint_name;
      if (!nh.getParam("joint", joint_name))
      {
        ROS_ERROR("Failed to get joint name for controller [%s]", controller_name.c_str());
        return false;
      }
      auto joint_urdf = urdf.getJoint(joint_name);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }

      if (controller_name == "yaw")
      {
        yaw_joint_urdf_ = joint_urdf;
        if (!yaw_vel_controller_.init(effort_joint_interface, nh) || !yaw_pos_pid_.init(nh_pid_pos))
          return false;
      }
      else if (controller_name == "pitch")
      {
        pitch_joint_urdf_ = joint_urdf;
        if (!pitch_vel_controller_.init(effort_joint_interface, nh) || !pitch_pos_pid_.init(nh_pid_pos))
          return false;
      }
    }
  }
  if (!yaw_joint_urdf_ || !pitch_joint_urdf_)
  {
    ROS_ERROR("Yaw/Picth joints are not fully initialized.");
    return false;
  }
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
}

}  // namespace rm_lidar_gimbal_controller

PLUGINLIB_EXPORT_CLASS(rm_lidar_gimbal_controller::Controller, controller_interface::ControllerBase)
