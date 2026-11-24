#pragma once

#include <cmath>
#include <ros/time.h>

#include "rm_chassis_controllers/bipedal_wheel_controller/definitions.h"

namespace rm_chassis_controllers
{
struct LegPosture
{
  double theta{ 0. }, rate{ 0. };
};

struct StandUpLegCommand
{
  double desired_length{};
  double desired_angle{};
  double desired_angle_vel{};
};

inline LegOrientation classifyStandUpLeg(const LegPosture& posture, const LegStateThresholdParams& thresholds,
                                        LegOrientation previous)
{
  if (posture.theta > thresholds.under_lower && posture.theta < thresholds.under_upper)
    return LegOrientation::UNDER;
  else if ((posture.theta < thresholds.front_lower && posture.theta > -M_PI) ||
           (posture.theta < M_PI && posture.theta > thresholds.front_upper))
    return LegOrientation::FRONT;
  else if (posture.theta > thresholds.behind_lower && posture.theta < thresholds.behind_upper)
    return LegOrientation::BEHIND;
  return previous;
}

inline void setUpLegMotion(const LegPosture& posture, const LegOrientation& other_leg_orientation,
                             const double& leg_length, const double& leg_theta, LegOrientation& leg_orientation,
                             StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, ros::Time& arrive_time,
                      const ros::Time& now, double arrive_time_threshold)
{
  switch (leg_orientation)
  {
    case LegOrientation::UNDER:
      stop_flag = false;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = 0.34;
      if (leg_length > 0.33)
      {
        leg_orientation = LegOrientation::FRONT;
      }
      break;
    case LegOrientation::FRONT:
      stop_flag = false;
      legCommand.desired_angle = M_PI_2 - 0.35;
      legCommand.desired_length = 0.34;
      legCommand.desired_angle_vel = 0.0;
      if (leg_length > 0.30)
        legCommand.desired_angle_vel = -5.0;
      if (abs(legCommand.desired_angle - leg_theta) < 0.5)
      {
        legCommand.desired_angle_vel = -1.5f;
      }
      if (abs(posture.rate) < 0.1)
      {
        if (posture.theta > 0 && posture.theta < M_PI_2 + 0.4f)
        {
          legCommand.desired_angle_vel = -0.5f;
          if (!arrive_flag)
          {
            arrive_flag = true;
            arrive_time = now;
          }
          if ((now - arrive_time).toSec() > arrive_time_threshold)
            leg_orientation = LegOrientation::BEHIND;
        }
      }
      break;
    case LegOrientation::BEHIND:
      stop_flag = true;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = leg_length;
      if (other_leg_orientation == LegOrientation::BEHIND)
      {
        stop_flag = false;

        legCommand.desired_length = 0.12f;
        legCommand.desired_angle = 0.0f;
      }
      break;
  }
}


inline bool standUpMotionComplete(double left_angle, double right_angle, LegOrientation left_orientation,
                                  LegOrientation right_orientation)
{
  return ((std::abs(left_angle) < .3 && left_orientation == LegOrientation::BEHIND) &&
       (std::abs(right_angle) < .3 && right_orientation == LegOrientation::BEHIND)) ||
      ((std::abs(left_angle) < .3 && left_orientation == LegOrientation::UNDER) &&
       (std::abs(right_angle) < .3 && right_orientation == LegOrientation::UNDER));
}
}  // namespace rm_chassis_controllers
