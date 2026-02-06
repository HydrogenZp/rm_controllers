/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by zhipeng on 2026/2/4
//

#pragma once

#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/filters/filters.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/GimbalPosState.h>
#include <urdf/model.h>
#include <dynamic_reconfigure/server.h>
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include "hardware_interface/robot_hw.h"
#include "rm_common/hardware_interface/rm_imu_sensor_interface.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <string>
namespace rm_lidar_gimbal_controller
{
class Controller
  : public controller_interface::MultiInterfaceController<
        hardware_interface::EffortJointInterface, rm_control::RobotStateInterface, rm_control::RmImuSensorInterface>
{
public:
  Controller() = default;
  ~Controller() override = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  void setDes(const ros::Time& time, double yaw_des, double pitch_des);
  bool setDesIntoLimit(const tf2::Quaternion& base2gimbal_des, const urdf::JointConstSharedPtr& joint_urdf,
                       tf2::Quaternion& base2new_des);
  void moveJoint(const ros::Time& time, const ros::Duration& period);

  void rate(const ros::Time& time, const ros::Duration& period);
  void track(const ros::Time& time, const ros::Duration& period);

private:
  void commandCB(const rm_msgs::GimbalCmdConstPtr& msg);
  rm_control::RobotStateHandle robot_state_handle_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;
  effort_controllers::JointVelocityController yaw_vel_controller_;
  effort_controllers::JointVelocityController pitch_vel_controller_;

  std::string imu_name_;

  control_toolbox::Pid yaw_pos_pid_;
  control_toolbox::Pid pitch_pos_pid_;
  urdf::JointConstSharedPtr yaw_joint_urdf_, pitch_joint_urdf_;

  ros::Subscriber cmd_sub_;
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_buffer_;
  rm_msgs::GimbalCmd gimbal_cmd_;

  // Transform
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2gimbal_, odom2base_, last_odom2base_;

  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>> yaw_state_pub_, pitch_state_pub_;
  double publish_rate_;
  ros::Time last_publish_time_;
  enum State
  {
    RATE,
    TRACK
  } state_ = RATE;
  bool has_imu_ = true;
  bool state_changed_ = true;
  bool start_ = true;
};

}  // namespace rm_lidar_gimbal_controller
