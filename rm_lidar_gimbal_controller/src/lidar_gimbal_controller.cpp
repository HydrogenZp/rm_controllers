#include "lidar_gimbal_controller.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/ori_tool.h>
#include "geometry_msgs/Vector3.h"
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "rm_common/hardware_interface/robot_state_interface.h"
#include "tf2/convert.h"

namespace rm_lidar_gimbal_controller
{
enum Axis
{
  ROLL = 0,
  PITCH = 1,
  YAW = 2
};

inline Axis getJointAxis(const urdf::JointConstSharedPtr& joint_urdf)
{
  if (joint_urdf->axis.x == 1)
    return ROLL;
  if (joint_urdf->axis.y == 1)
    return PITCH;
  return YAW;
}

bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  if (!controller_nh.getParam("imu_name", imu_name_))
  {
    ROS_ERROR("Param imu_name not set.");
    return false;
  }
  auto* imu_interface = robot_hw->get<rm_control::RmImuSensorInterface>();
  if (!imu_interface)
  {
    ROS_ERROR("Failed to get RmImuSensorInterface.");
    return false;
  }
  imu_sensor_handle_ = imu_interface->getHandle(imu_name_);

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

  cmd_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this,
                                                         ros::TransportHints().reliable().tcpNoDelay());
  return true;
}

void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  gimbal_cmd_ = *cmd_buffer_.readFromRT();
}

void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base);
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
  tf2::Quaternion base2gimbal_des = odom2base.inverse() * odom2gimbal_des;

  // Check yaw limit
  setDesIntoLimit(base2gimbal_des, yaw_joint_urdf_, base2gimbal_des);
  // Check pitch limit
  setDesIntoLimit(base2gimbal_des, pitch_joint_urdf_, base2gimbal_des);

  odom2gimbal_des_.transform.rotation = tf2::toMsg(odom2base * base2gimbal_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_lidar_gimbal_controller");
}

bool Controller::setDesIntoLimit(const tf2::Quaternion& base2gimbal_des, const urdf::JointConstSharedPtr& joint_urdf,
                                 tf2::Quaternion& base2new_des)
{
  double base2gimbal_current_des[3];
  quatToRPY(tf2::toMsg(base2gimbal_des), base2gimbal_current_des[0], base2gimbal_current_des[1],
            base2gimbal_current_des[2]);

  double upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
  double lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;

  // Determine which axis this joint controls
  Axis axis = getJointAxis(joint_urdf);

  if ((base2gimbal_current_des[axis] <= upper_limit && base2gimbal_current_des[axis] >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des[axis]) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des[axis]) >= lower_limit))
  {
    base2new_des = base2gimbal_des;
    return true;
  }
  else
  {
    // Clamp to the nearest limit
    base2gimbal_current_des[axis] =
        std::abs(angles::shortest_angular_distance(base2gimbal_current_des[axis], upper_limit)) <
                std::abs(angles::shortest_angular_distance(base2gimbal_current_des[axis], lower_limit)) ?
            upper_limit :
            lower_limit;
    base2new_des.setRPY(base2gimbal_current_des[ROLL], base2gimbal_current_des[PITCH], base2gimbal_current_des[YAW]);
    return false;
  }
}

void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[LidarGimbal] Enter RATE");
    if (start_)
    {
      odom2gimbal_des_.transform.rotation = odom2gimbal_.transform.rotation;
      odom2gimbal_des_.header.stamp = time;
      robot_state_handle_.setTransform(odom2gimbal_des_, "rm_lidar_gimbal_controller");
      start_ = false;
    }
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
    setDes(time, yaw + period.toSec() * gimbal_cmd_.rate_yaw, pitch + period.toSec() * gimbal_cmd_.rate_pitch);
  }
}

void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel;
  if (has_imu_)
  {
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      tf2::doTransform(gyro, angular_vel,
                       robot_state_handle_.lookupTransform(odom2gimbal_.child_frame_id, imu_sensor_handle_.getFrameId(),
                                                           time));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  else
  {
    angular_vel.y = pitch_vel_controller_.joint_.getVelocity();
    angular_vel.z = yaw_vel_controller_.joint_.getVelocity();
  }

  double pos_real[3]{ 0. }, pos_des[3]{ 0. }, vel_des[3]{ 0. }, angle_error[3]{ 0. };
  quatToRPY(odom2gimbal_des_.transform.rotation, pos_des[0], pos_des[1], pos_des[2]);
  quatToRPY(odom2gimbal_.transform.rotation, pos_real[0], pos_real[1], pos_real[2]);

  for (int i = 0; i < 3; i++)
    angle_error[i] = angles::shortest_angular_distance(pos_real[i], pos_des[i]);

  if (state_ == RATE)
  {
    vel_des[2] = gimbal_cmd_.rate_yaw;
    vel_des[1] = gimbal_cmd_.rate_pitch;
  }

  yaw_pos_pid_.computeCommand(angle_error[YAW], period);
  pitch_pos_pid_.computeCommand(angle_error[PITCH], period);

  yaw_vel_controller_.setCommand(yaw_pos_pid_.getCurrentCmd() + vel_des[YAW] +
                                 yaw_vel_controller_.joint_.getVelocity() - angular_vel.z);
  pitch_vel_controller_.setCommand(pitch_pos_pid_.getCurrentCmd() + vel_des[PITCH] +
                                   pitch_vel_controller_.joint_.getVelocity() - angular_vel.y);

  yaw_vel_controller_.update(time, period);
  pitch_vel_controller_.update(time, period);
}

void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_buffer_.writeFromNonRT(*msg);
}
}  // namespace rm_lidar_gimbal_controller

PLUGINLIB_EXPORT_CLASS(rm_lidar_gimbal_controller::Controller, controller_interface::ControllerBase)
