#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/bipedal_wheel_controller/controller_mode/stand_up_motion.h"
#include <hardware_interface/imu_sensor_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/LegCmd.h>
#include <rm_msgs/LeggedChassisMode.h>
#include <std_msgs/String.h>
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace rm_chassis_controllers
{
namespace
{
using Clock = std::chrono::steady_clock;
using Action = std::array<float, 6>;
using Frame = std::array<float, 35>;
using Observation = std::array<float, 210>;
constexpr std::array<double, 4> reference{{.652747116948, -.674200211242, .652747116948, -.674200211242}};
constexpr std::array<const char*, 6> motor_names{{"right_hip_thigh", "right_hip_output", "left_hip_thigh",
                                                  "left_hip_output", "right_wheel_joint", "left_wheel_joint"}};

template <typename Range>
bool finite(const Range& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

double limit(double value, double maximum)
{
  return std::max(-maximum, std::min(maximum, value));
}

struct LegGeometry
{
  double length;
  double delta;
  double angle_offset;
};

constexpr LegGeometry geometry[] = {
#include "stand_up_geometry.inc"
};

LegGeometry interpolate(double value, double LegGeometry::*key)
{
  const auto direction = (geometry[0].*key < geometry[1].*key) ? 1.0 : -1.0;
  const auto* upper = std::lower_bound(std::begin(geometry), std::end(geometry), value,
      [key, direction](const LegGeometry& point, double query) { return direction * (point.*key - query) < 0; });
  if (upper == std::begin(geometry))
    return *upper;
  if (upper == std::end(geometry))
    return *(upper - 1);
  const auto& lower = *(upper - 1);
  const double weight = (value - lower.*key) / ((*upper).*key - lower.*key);
  return {lower.length + weight * (upper->length - lower.length),
          lower.delta + weight * (upper->delta - lower.delta),
          lower.angle_offset + weight * (upper->angle_offset - lower.angle_offset)};
}

class Policy
{
public:
  explicit Policy(const std::string& path)
  {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    session_ = Ort::Session(environment_, path.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    const auto id = session_.GetModelMetadata().LookupCustomMetadataMapAllocated("serial_leg_interface_id", allocator);
    if (!id || std::string(id.get()) != "serial_leg_history5_100hz_v1" ||
        session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
      throw std::runtime_error("Incompatible V16 policy metadata");
    const auto check = [](const Ort::TypeInfo& type, const Ort::AllocatedStringPtr& name,
                                    const char* expected_name, int size) {
      const auto tensor = type.GetTensorTypeAndShapeInfo();
      if (std::string(name.get()) != expected_name || tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
          tensor.GetShape() != std::vector<int64_t>{1, size})
        throw std::runtime_error("Incompatible V16 tensor");
    };
    check(session_.GetInputTypeInfo(0), session_.GetInputNameAllocated(0, allocator), "obs", 210);
    check(session_.GetOutputTypeInfo(0), session_.GetOutputNameAllocated(0, allocator), "actions", 6);
    Observation zero{};
    run(zero);
  }

  Action run(Observation& input)
  {
    const std::array<int64_t, 2> input_shape{{1, 210}}, output_shape{{1, 6}};
    const char* input_name = "obs";
    const char* output_name = "actions";
    Action result{};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto source = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), input_shape.data(), 2);
    auto destination = Ort::Value::CreateTensor<float>(memory, result.data(), result.size(), output_shape.data(), 2);
    session_.Run(Ort::RunOptions{nullptr}, &input_name, &source, 1, &output_name, &destination, 1);
    if (!finite(result))
      throw std::runtime_error("V16 inference produced non-finite output");
    return result;
  }

private:
  Ort::Env environment_{ORT_LOGGING_LEVEL_WARNING, "serial_leg_rl"};
  Ort::Session session_{nullptr};
};

struct StandLeg
{
  LegOrientation orientation{BEHIND};
  StandUpLegCommand target;
  LegGeometry measured{};
  LegPosture posture;
  ros::Time arrival;
  bool stopped{}, arrived{};
};
}  // namespace

class SerialLegRlController : public ChassisBase<rm_control::RobotStateInterface,
                                                  hardware_interface::ImuSensorInterface,
                                                  hardware_interface::EffortJointInterface>
{
public:
  ~SerialLegRlController() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quit_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable())
      worker_.join();
  }

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root, ros::NodeHandle& nh) override
  {
    if (!ChassisBase::init(hw, root, nh))
      return false;
    cmd_struct_.cmd_chassis_.mode = rm_msgs::ChassisCmd::FALLEN;
    cmd_rt_buffer_.initRT(cmd_struct_);
    try
    {
      std::string path;
      if (!nh.getParam("policy_path", path) || path.empty())
        throw std::runtime_error("policy_path is required");
      nh.param("inference_timeout", inference_timeout_, .040);
      if (!std::isfinite(inference_timeout_) || inference_timeout_ <= 0.)
        throw std::runtime_error("inference_timeout must be positive");
      const std::pair<const char*, double*> params[] = {
          {"under_lower_threshold", &thresholds_.under_lower}, {"under_upper_threshold", &thresholds_.under_upper},
          {"front_lower_threshold", &thresholds_.front_lower}, {"front_upper_threshold", &thresholds_.front_upper},
          {"behind_lower_threshold", &thresholds_.behind_lower}, {"behind_upper_threshold", &thresholds_.behind_upper},
          {"arrive_time_threshold", &thresholds_.arrive_time_threshold}};
      for (const auto& param : params)
        if (!nh.getParam(param.first, *param.second))
          throw std::runtime_error(std::string("Missing stand-up parameter ") + param.first);
      for (const auto* name : motor_names)
        motors_.push_back(effort_joint_interface_->getHandle(std::string("policy_") + name));
      imu_ = hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
      policy_.reset(new Policy(path));
      leg_sub_ = root.subscribe<rm_msgs::LegCmd>("leg_cmd", 1, [this](const rm_msgs::LegCmd::ConstPtr& msg) {
        if (std::isfinite(msg->leg_length))
          leg_length_.writeFromNonRT(msg->leg_length);
      });
      modes_.reset(new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>(nh, "legged_chassis_mode", 4));
      faults_ = nh.advertise<std_msgs::String>("policy_fault", 1, true);
      worker_ = std::thread(&SerialLegRlController::inferenceLoop, this);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("SerialLegRlController: " << e.what());
      return false;
    }
    return true;
  }

  void stopping(const ros::Time&) override
  {
    zero();
    resetMotion();
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    if (faulted_)
    {
      zero();
      return;
    }
    try
    {
      ChassisBase::update(time, period);
    }
    catch (const std::exception& error)
    {
      zero();
      faulted_ = true;
      ROS_ERROR_STREAM("SerialLegRlController exception: " << error.what());
      std_msgs::String msg;
      msg.data = error.what();
      faults_.publish(msg);
      publishMode(PROTECT, "FAULT");
      // Return to RobotHW so its write phase commits the six zero commands.
    }
  }

protected:
  void powerLimit() override {}  // The motor limits below apply to the experimental policy and stand-up.
  geometry_msgs::Twist odometry() override
  {
    geometry_msgs::Twist result;
    result.linear.x = forward_speed_;
    result.angular.z = gyro_[2];
    return result;
  }

  void moveJoint(const ros::Time& time, const ros::Duration& period) override
  {
    if (state_ != rm_msgs::ChassisCmd::RAW)
    {
      zero();
      resetMotion();
      publishMode(SIT_DOWN, "SIT_DOWN");
      return;
    }

    const double dt = period.toSec();
    if (!std::isfinite(dt) || dt <= 0.0)
      throw std::runtime_error("Invalid control period");
    readState();
    const bool new_stand = phase_ == Phase::Sitting;
    if (new_stand)
    {
      if (std::abs(gyro_[1]) >= .1)
        return;
      enterStand(time);
    }
    if (gravity_[2] > -.42 || time < phase_start_)
      throw std::runtime_error("Invalid tilt or time during RL control");

    std::array<double, 6> desired{};
    if (phase_ == Phase::Policy)
    {
      updatePolicy(time);
      for (std::size_t i = 0; i < 4; ++i)
      {
        const double target = reference[i] + 1.2 * limit(action_[i], 1.0);
        target_[i] += limit(target - target_[i], 4.0 * dt);
        desired[i] = 60.0 * (target_[i] - position_[i]) - 2.0 * velocity_[i];
      }
      for (std::size_t i = 4; i < 6; ++i)
        desired[i] = .2 * (limit(10.0 * action_[i], 150.0) - velocity_[i]);
    }
    else
    {
      if ((time - stand_start_).toSec() > 8.0)
        throw std::runtime_error("Serial-leg stand-up timed out");
      standUp(time, dt);
      for (std::size_t i = 0; i < 4; ++i)
        desired[i] = 85.0 * (target_[i] - position_[i]) - 3.0 * velocity_[i];
    }

    if (!finite(desired))
      throw std::runtime_error("Invalid PD input or output");
    for (std::size_t i = 0; i < motors_.size(); ++i)
      motors_[i].setCommand(limit(desired[i], i < 4 ? 40.0 : 5.0));
    publishMode(phase_ == Phase::Policy ? NORMAL : STAND_UP, phase_ == Phase::Policy ? "NORMAL" : "STAND_UP");
  }

private:
  enum class Phase { Sitting, StandUp, Extend, Swing, Rise, Policy };

  void zero()
  {
    for (auto& motor : motors_)
      motor.setCommand(0.0);
  }

  void resetMotion()
  {
    if (phase_ == Phase::Sitting)
      return;
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    request_pending_ = false;
    result_ready_ = false;
    awaiting_ = false;
    first_frame_ = true;
    action_.fill(0.f);
    phase_ = Phase::Sitting;
  }

  void readState()
  {
    for (std::size_t i = 0; i < motors_.size(); ++i)
    {
      position_[i] = motors_[i].getPosition();
      velocity_[i] = motors_[i].getVelocity();
    }
    const auto* q = imu_.getOrientation();
    tf2::Quaternion rotation(q[0], q[1], q[2], q[3]);
    if (!finite(position_) || !finite(velocity_) || !std::isfinite(rotation.length2()) ||
        std::abs(rotation.length2() - 1.) > .05)
      throw std::runtime_error("Invalid motor or IMU state");
    rotation.normalize();
    const auto down = tf2::quatRotate(rotation.inverse(), tf2::Vector3(0, 0, -1));
    gravity_ = {{down.x(), down.y(), down.z()}};
    std::copy_n(imu_.getAngularVelocity(), 3, gyro_.begin());
    if (!finite(gyro_))
      throw std::runtime_error("Invalid IMU angular velocity");
    pitch_ = std::atan2(gravity_[0], -gravity_[2]);
    forward_speed_ = 0.0;
    for (std::size_t side = 0; side < legs_.size(); ++side)
    {
      const auto hip = 2 * side;
      const auto leg = interpolate(position_[hip + 1] - position_[hip], &LegGeometry::delta);
      const double angle = -position_[hip] - leg.angle_offset;
      const double delta_rate = velocity_[hip + 1] - velocity_[hip];
      const auto next = interpolate(leg.delta + .001 * delta_rate, &LegGeometry::delta);
      const auto previous = interpolate(leg.delta - .001 * delta_rate, &LegGeometry::delta);
      const double swing_rate = -velocity_[hip] - (next.angle_offset - previous.angle_offset) / .002;
      legs_[side].measured = {leg.length, leg.delta, angle};
      legs_[side].posture = {std::remainder(angle + pitch_, 2 * M_PI), swing_rate + gyro_[1]};
      // Wheel encoders report relative rotation about the MJCF +Y axis.
      const double dlength = (next.length - previous.length) / .002;
      const double calf_rate = swing_rate +
          std::acos(limit((.21 * .21 - .248 * .248 - next.length * next.length) / (.496 * next.length), 1.0)) / .002 -
          std::acos(limit((.21 * .21 - .248 * .248 - previous.length * previous.length) / (.496 * previous.length), 1.0)) / .002;
      forward_speed_ += .5 * (.061 * (gyro_[1] + calf_rate - velocity_[4 + side]) +
                              dlength * std::sin(angle) + leg.length * std::cos(angle) * swing_rate);
    }
  }

  void enterStand(const ros::Time& time)
  {
    stand_start_ = phase_start_ = time;
    rise_ready_since_ = ros::Time();
    phase_ = Phase::StandUp;
    const bool folded = std::all_of(legs_.begin(), legs_.end(), [](const StandLeg& leg) {
      return leg.measured.length < .19 && leg.measured.angle_offset > 1.1;
    });
    if (folded)
      phase_ = Phase::Extend;
    for (std::size_t side = 0; side < legs_.size(); ++side)
    {
      auto& leg = legs_[side];
      leg.orientation = classifyStandUpLeg(leg.posture, thresholds_, BEHIND);
      leg.target = {leg.measured.length, leg.measured.angle_offset, 0.0};
      rise_length_[side] = leg.measured.length;
      rise_angle_[side] = leg.measured.angle_offset;
      leg.stopped = leg.arrived = false;
      leg.arrival = ros::Time();
      std::copy_n(position_.begin() + 2 * side, 2, target_.begin() + 2 * side);
    }
  }

  void standUp(const ros::Time& time, double dt)
  {
    const double elapsed = (time - phase_start_).toSec();
    for (std::size_t side = 0; side < legs_.size(); ++side)
    {
      auto& leg = legs_[side];
      if (phase_ == Phase::StandUp)
      {
        setUpLegMotion(leg.posture, legs_[1 - side].orientation, leg.measured.length, leg.measured.angle_offset,
                       leg.orientation, leg.target, leg.stopped, leg.arrived, leg.arrival, time,
                       thresholds_.arrive_time_threshold);
        if (leg.stopped)
          continue;
        if (leg.orientation == FRONT && leg.target.desired_angle_vel < 0.0)
          leg.target.desired_angle = leg.measured.angle_offset + .15 * leg.target.desired_angle_vel;
      }
      else if (phase_ == Phase::Extend || phase_ == Phase::Swing)
      {
        const bool extend = phase_ == Phase::Extend;
        const double u = std::min(elapsed / (extend ? .28 : .55), 1.0);
        const double smooth = u * u * (3.0 - 2.0 * u);
        leg.target.desired_length = extend ? rise_length_[side] + (.21 - rise_length_[side]) * smooth : .21;
        leg.target.desired_angle = extend ? rise_angle_[side] : rise_angle_[side] + (.25 - rise_angle_[side]) * smooth;
      }
      else
      {
        const double progress = std::min(elapsed / .65, 1.0);
        const double smooth = progress * progress * (3.0 - 2.0 * progress);
        leg.target.desired_length = rise_length_[side] + (.259 - rise_length_[side]) * smooth;
        leg.target.desired_angle = rise_angle_[side] * (1.0 - smooth) + .012 * smooth -
            limit(1.65 * pitch_ + .20 * gyro_[1], .85);
      }
      const auto desired = interpolate(leg.target.desired_length, &LegGeometry::length);
      const double thigh = -leg.target.desired_angle - desired.angle_offset;
      if (phase_ == Phase::StandUp)
      {
        target_[2 * side] += limit(thigh - target_[2 * side], 5.0 * dt);
        target_[2 * side + 1] += limit(thigh + desired.delta - target_[2 * side + 1], 5.0 * dt);
      }
      else
      {
        target_[2 * side] = thigh;
        target_[2 * side + 1] = thigh + desired.delta;
      }
    }

    if (phase_ == Phase::Extend && elapsed >= .28)
    {
      phase_ = Phase::Swing;
      phase_start_ = time;
    }
    else if (phase_ == Phase::Swing && elapsed >= .56)
    {
      phase_ = Phase::Rise;
      phase_start_ = time;
      rise_length_.fill(.21);
      rise_angle_.fill(.25);
    }
    else if (phase_ == Phase::StandUp && standUpMotionComplete(
          legs_[1].measured.angle_offset, legs_[0].measured.angle_offset, legs_[1].orientation, legs_[0].orientation))
    {
      phase_ = Phase::Rise;
      phase_start_ = time;
      for (std::size_t side = 0; side < legs_.size(); ++side)
      {
        rise_length_[side] = legs_[side].measured.length;
        rise_angle_[side] = legs_[side].measured.angle_offset;
      }
    }
    else if (phase_ == Phase::Rise && elapsed >= .65)
    {
      const bool ready = gravity_[2] < -std::cos(.21) && std::hypot(gyro_[0], gyro_[1]) < .70 &&
                         std::abs(forward_speed_) < .20 &&
                         legs_[0].measured.length > .18 && legs_[1].measured.length > .18 &&
                         std::equal(reference.begin(), reference.end(), position_.begin(),
                                    [](double target, double actual) { return std::abs(target - actual) < .30; });
      if (!ready)
        rise_ready_since_ = ros::Time();
      else if (rise_ready_since_.isZero())
        rise_ready_since_ = time;
      else if ((time - rise_ready_since_).toSec() >= .06)
      {
        phase_ = Phase::Policy;
        phase_start_ = time;
        next_inference_ = time;
        handoff_ = time;
        ROS_INFO("SerialLegRlController handed stand-up over to V16");
      }
    }
  }

  void updatePolicy(const ros::Time& time)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (awaiting_ && result_ready_)
    {
      awaiting_ = false;
      result_ready_ = false;
      if (!result_error_.empty())
        throw std::runtime_error(result_error_);
      if (std::chrono::duration<double>(completed_at_ - submitted_at_).count() > inference_timeout_)
        throw std::runtime_error("V16 inference timed out");
      action_ = result_;
    }
    if (awaiting_ && std::chrono::duration<double>(Clock::now() - submitted_at_).count() > inference_timeout_)
      throw std::runtime_error("V16 inference timed out");
    if (awaiting_ || time < next_inference_)
      return;

    Frame frame{};
    const auto& velocity = vel_cmd_;
    const bool settling = (time - handoff_).toSec() < 1.0;
    frame[0] = settling ? 0.f : limit(velocity.x, 3.0);
    frame[2] = settling ? 0.f : limit(velocity.z, 2 * M_PI);
    frame[3] = 5.0 * (settling ? .259 : std::max(.15, std::min(.33, *leg_length_.readFromRT())));
    for (std::size_t i = 0; i < 3; ++i)
    {
      frame[4 + i] = .5 * gyro_[i];
      frame[7 + i] = gravity_[i];
    }
    for (std::size_t i = 0; i < 4; ++i)
      frame[10 + i] = position_[i] - reference[i];
    for (std::size_t i = 0; i < 6; ++i)
    {
      frame[16 + i] = .1 * velocity_[i];
      frame[22 + i] = action_[i];
    }
    frame[28] = 1.f;
    if (!finite(frame))
      throw std::runtime_error("Non-finite V16 observation");
    // Match simulation.py: current frame followed by five historical frames, oldest first.
    if (first_frame_)
    {
      history_.fill(frame);
      first_frame_ = false;
    }
    else
    {
      std::move(history_.begin() + 1, history_.end(), history_.begin());
      history_.back() = frame;
    }
    std::copy(frame.begin(), frame.end(), input_.begin());
    for (std::size_t i = 0; i < history_.size(); ++i)
      std::copy(history_[i].begin(), history_[i].end(), input_.begin() + (i + 1) * frame.size());
    submitted_at_ = Clock::now();
    awaiting_ = request_pending_ = true;
    next_inference_ = time + ros::Duration(.01);
    wake_.notify_one();
  }

  void inferenceLoop()
  {
    while (true)
    {
      Observation input;
      unsigned generation;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return quit_ || request_pending_; });
        if (quit_)
          return;
        input = input_;
        generation = generation_;
        request_pending_ = false;
      }
      Action result{};
      std::string error;
      try
      {
        result = policy_->run(input);
      }
      catch (const std::exception& e)
      {
        error = e.what();
      }
      const auto completed = Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation == generation_)
      {
        result_ = result;
        result_error_ = std::move(error);
        completed_at_ = completed;
        result_ready_ = true;
      }
    }
  }

  void publishMode(uint8_t mode, const char* name)
  {
    if (modes_->trylock())
    {
      modes_->msg_.mode = mode;
      modes_->msg_.mode_name = name;
      modes_->unlockAndPublish();
    }
  }

  Phase phase_{Phase::Sitting};
  std::array<StandLeg, 2> legs_{};
  LegStateThresholdParams thresholds_{};
  std::array<double, 6> position_{}, velocity_{};
  std::array<double, 4> target_{};
  std::array<double, 3> gyro_{}, gravity_{};
  std::array<double, 2> rise_length_{}, rise_angle_{};
  double pitch_{}, forward_speed_{}, inference_timeout_{};
  ros::Time phase_start_, stand_start_, rise_ready_since_, handoff_, next_inference_;
  bool faulted_{};
  std::vector<hardware_interface::JointHandle> motors_;
  hardware_interface::ImuSensorHandle imu_;
  ros::Subscriber leg_sub_;
  ros::Publisher faults_;
  realtime_tools::RealtimeBuffer<double> leg_length_{.259};
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>> modes_;
  std::unique_ptr<Policy> policy_;
  std::array<Frame, 5> history_{};
  Action action_{}, result_{};
  Observation input_{};
  std::string result_error_;
  Clock::time_point submitted_at_, completed_at_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  unsigned generation_{};
  bool quit_{}, request_pending_{}, result_ready_{}, awaiting_{}, first_frame_{true};
};
}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::SerialLegRlController, controller_interface::ControllerBase)
