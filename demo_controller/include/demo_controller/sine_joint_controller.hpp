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

#pragma once

#include <memory>

#include "cho_controller_franka/base_controller.hpp"
#include "demo_controller/demo_sine_joint_controller_parameters.hpp"
#include "demo_controller/sine_reference.hpp"

namespace demo_controller
{

// A project's own arm controller on the core's FrankaBaseController: in the
// position control mode, it sweeps one joint in a sine about where the arm was
// when it was switched in, and holds every other joint there. It serves no
// action: a task switches it in, lets it run, and switches the hold controller
// back in.
//
// What it takes from the base class, as every core Franka controller does:
// - held_command_position(): the position the previous controller left on the
//   command interfaces, where the sweep is centred;
// - nominal_period(): the sweep's clock;
// - clip_position() and limit_position_command() on every command it writes;
// - reference_motion(): the sweep's rate and acceleration when it is switched
//   out, which the hold controller switched in takes over and brings to rest
//   (held_command_motion()), instead of stopping the joint in one period.
//
// It refuses to activate on an arm that is still moving, or where the sweep
// would leave the joint's limits.
class SineJointController : public cho_controller::franka::FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  bool reference_motion(Vector7d & rate, Vector7d & acceleration) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<demo_sine_joint_controller::ParamListener> param_listener_;
  demo_sine_joint_controller::Params params_;
  int joint_{6};             // 0-based index of params_.joint
  Vector7d center_{Vector7d::Zero()};
  Vector7d last_cmd_{Vector7d::Zero()};
  double clock_{0.0};        // [s] since activation, advanced by nominal_period()
  SineSample sample_;        // the last command's, for reference_motion()
};

}  // namespace demo_controller
