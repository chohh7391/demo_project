// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "demo_controller/sine_joint_controller.hpp"

#include <exception>
#include <string>

namespace demo_controller
{

controller_interface::InterfaceConfiguration
SineJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

CallbackReturn SineJointController::on_init()
{
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  try {
    // Declares and range-checks every parameter in
    // sine_joint_controller_parameters.yaml; a value outside its range throws.
    param_listener_ = std::make_shared<demo_sine_joint_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJointController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  params_ = param_listener_->get_params();
  joint_ = static_cast<int>(params_.joint) - 1;
  // The one check that relates parameters: each is in range on its own.
  const double rate = peak_rate(params_.amplitude, params_.frequency);
  if (rate > params_.max_rate) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "amplitude %.3f rad at %.3f Hz peaks at %.3f rad/s, above max_rate %.3f rad/s",
      params_.amplitude, params_.frequency, rate, params_.max_rate);
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SineJointController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  // A moving command would be stopped in this controller's first period: the
  // sweep starts from rest, so it takes over only an arm at rest.
  Vector7d rate, acceleration;
  if (held_command_motion(rate, acceleration)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "the controller switched out left the arm moving; switch in from an arm at rest");
    return CallbackReturn::ERROR;
  }
  center_ = held_command_position();
  const double low = center_(joint_) - params_.amplitude;
  const double high = center_(joint_) + params_.amplitude;
  if (low < q_lower_limits_(joint_) || high > q_upper_limits_(joint_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "joint %ld swept over [%.3f, %.3f] rad leaves its limits [%.3f, %.3f] rad",
      params_.joint, low, high, q_lower_limits_(joint_), q_upper_limits_(joint_));
    return CallbackReturn::ERROR;
  }
  // clip_position() limits each step around q_arm_ref: start it at the centre.
  state_.q_arm_ref = center_;
  state_.q_arm_des = center_;
  last_cmd_ = center_;
  clock_ = 0.0;
  sample_ = SineSample{};
  return CallbackReturn::SUCCESS;
}

bool SineJointController::reference_motion(Vector7d & rate, Vector7d & acceleration)
{
  rate.setZero();
  acceleration.setZero();
  rate(joint_) = sample_.rate;
  acceleration(joint_) = sample_.acceleration;
  return true;
}

controller_interface::return_type SineJointController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }
  clock_ += nominal_period(period);
  const SineSample sample =
    sine_sample(clock_, params_.amplitude, params_.frequency, params_.ramp_sec);

  Vector7d q_cmd = center_;
  q_cmd(joint_) += sample.offset;
  if (!q_cmd.allFinite()) {
    // A position interface holds the previous command.
    q_cmd = last_cmd_;
  } else {
    sample_ = sample;
  }
  state_.q_arm_des = q_cmd;
  clip_position(q_cmd);
  limit_position_command(q_cmd);
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_cmd(i));
  }
  last_cmd_ = q_cmd;
  return controller_interface::return_type::OK;
}

}  // namespace demo_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(demo_controller::SineJointController, controller_interface::ControllerInterface)
