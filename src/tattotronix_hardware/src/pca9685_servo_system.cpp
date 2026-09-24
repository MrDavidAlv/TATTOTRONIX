// Copyright 2026 Mario David Alvarez Vallejo
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

#include "tattotronix_hardware/pca9685_servo_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace tattotronix_hardware
{

namespace
{

using Params = std::unordered_map<std::string, std::string>;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("Pca9685ServoSystem");
}

/// The whole string as a number, or false. "12abc" is not 12.
bool to_number(const std::string & text, double & value)
{
  try {
    std::size_t used = 0;
    const double parsed = std::stod(text, &used);
    if (used != text.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

/// A parameter that may be absent, in which case `value` keeps its default.
bool optional_number(const Params & params, const std::string & key, double & value)
{
  const auto found = params.find(key);
  return found == params.end() || to_number(found->second, value);
}

bool is_true(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return text == "true" || text == "1";
}

}  // namespace

Pca9685ServoSystem::~Pca9685ServoSystem()
{
  stop();
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const Params & hw = info.hardware_parameters;
  double address = address_;
  if (!optional_number(hw, "frame_rate_hz", frame_hz_) ||
    !optional_number(hw, "oscillator_hz", oscillator_hz_) ||
    !optional_number(hw, "i2c_address", address))
  {
    RCLCPP_ERROR(logger(), "frame_rate_hz, oscillator_hz and i2c_address must be numbers");
    return CallbackReturn::ERROR;
  }
  if (!(frame_hz_ > 0.0) || !(oscillator_hz_ > 0.0) || address < 0 || address > 0x7F) {
    RCLCPP_ERROR(logger(), "frame rate and oscillator must be positive, the address 0 to 127");
    return CallbackReturn::ERROR;
  }
  address_ = static_cast<int>(address);
  if (hw.count("i2c_device")) {
    device_ = hw.at("i2c_device");
  }
  if (hw.count("dry_run")) {
    dry_run_ = is_true(hw.at("dry_run"));
  }

  std::vector<int> used;
  joints_.clear();
  joints_.reserve(info.joints.size());
  for (const auto & component : info.joints) {
    Joint joint;
    joint.name = component.name;
    const char * name = joint.name.c_str();

    if (component.command_interfaces.size() != 1 ||
      component.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger(), "joint '%s' must have one command interface, position: a hobby servo "
        "takes nothing else", name);
      return CallbackReturn::ERROR;
    }
    const auto & command = component.command_interfaces[0];
    if ((!command.min.empty() && !to_number(command.min, joint.lower)) ||
      (!command.max.empty() && !to_number(command.max, joint.upper)) ||
      !(joint.lower < joint.upper))
    {
      RCLCPP_ERROR(logger(), "joint '%s' has command limits that are not a range", name);
      return CallbackReturn::ERROR;
    }

    bool has_position = false;
    for (const auto & state : component.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
        if (!state.initial_value.empty() && !to_number(state.initial_value, joint.position)) {
          RCLCPP_ERROR(logger(), "joint '%s' has an initial position that is not a number", name);
          return CallbackReturn::ERROR;
        }
      } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
        joint.has_velocity = true;
      } else {
        RCLCPP_ERROR(
          logger(), "joint '%s' asks for a '%s' state, which a hobby servo cannot report",
          name, state.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (!has_position) {
      RCLCPP_ERROR(logger(), "joint '%s' needs a position state interface", name);
      return CallbackReturn::ERROR;
    }

    // One servo per joint, or two sharing one: the second is the "_b" set, and
    // shares the first's pulse limits.
    for (const std::string suffix : {"", "_b"}) {
      if (!component.parameters.count("channel" + suffix)) {
        if (suffix.empty()) {
          RCLCPP_ERROR(logger(), "joint '%s' names no PCA9685 channel", name);
          return CallbackReturn::ERROR;
        }
        break;
      }
      ServoChannel servo;
      double channel = -1.0;
      if (!optional_number(component.parameters, "channel" + suffix, channel) ||
        !optional_number(component.parameters, "zero_us" + suffix, servo.zero_us) ||
        !optional_number(component.parameters, "us_per_rad" + suffix, servo.us_per_rad) ||
        !optional_number(component.parameters, "min_us", servo.min_us) ||
        !optional_number(component.parameters, "max_us", servo.max_us))
      {
        RCLCPP_ERROR(logger(), "joint '%s' has a servo parameter that is not a number", name);
        return CallbackReturn::ERROR;
      }
      servo.channel = static_cast<int>(channel);
      if (*servo.problem()) {
        RCLCPP_ERROR(
          logger(), "joint '%s', channel%s: %s", name, suffix.c_str(), servo.problem());
        return CallbackReturn::ERROR;
      }
      if (std::find(used.begin(), used.end(), servo.channel) != used.end()) {
        RCLCPP_ERROR(
          logger(), "joint '%s': channel %d already drives another servo", name, servo.channel);
        return CallbackReturn::ERROR;
      }
      used.push_back(servo.channel);
      joint.servos.push_back(servo);
      joint.sent.push_back(-1);
    }

    joint.position = std::clamp(joint.position, joint.lower, joint.upper);
    joint.command = joint.position;
    joint.previous = joint.position;
    joints_.push_back(joint);
  }
  return CallbackReturn::SUCCESS;
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    if (dry_run_) {
      bus_ = std::make_unique<RecordingBus>();
    } else {
      bus_ = std::make_unique<LinuxI2cBus>(device_, static_cast<uint8_t>(address_));
    }
    board_ = std::make_unique<Pca9685>(*bus_, frame_hz_, oscillator_hz_);
    board_->start();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "cannot start the PCA9685: %s", error.what());
    board_.reset();
    bus_.reset();
    return CallbackReturn::ERROR;
  }
  const std::string where = dry_run_ ? std::string("in a dry run, sending nothing") :
    "on " + device_ + " at address " + std::to_string(address_);
  RCLCPP_INFO(
    logger(), "PCA9685 %s: %.1f Hz frames, one count is %.2f us. Every channel is off.",
    where.c_str(), frame_hz_, board_->count_us());
  return CallbackReturn::SUCCESS;
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    for (auto & joint : joints_) {
      joint.command = joint.position;
      std::fill(joint.sent.begin(), joint.sent.end(), -1);
      send(joint);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "cannot command the servos: %s", error.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_WARN(
    logger(), "Servos powered. Each one goes at full speed to the pose it was sent, so the "
    "arm has to be near that pose already.");
  return CallbackReturn::SUCCESS;
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop();
  return CallbackReturn::SUCCESS;
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop();
  board_.reset();
  bus_.reset();
  return CallbackReturn::SUCCESS;
}

Pca9685ServoSystem::CallbackReturn Pca9685ServoSystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> Pca9685ServoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    if (joint.has_velocity) {
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Pca9685ServoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.command);
  }
  return interfaces;
}

hardware_interface::return_type Pca9685ServoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (auto & joint : joints_) {
    joint.velocity = dt > 0.0 ? (joint.position - joint.previous) / dt : 0.0;
    joint.previous = joint.position;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685ServoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!board_) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    for (auto & joint : joints_) {
      if (!std::isnan(joint.command)) {
        send(joint);
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "lost the PCA9685: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

const RecordingBus * Pca9685ServoSystem::recording() const
{
  return dynamic_cast<const RecordingBus *>(bus_.get());
}

void Pca9685ServoSystem::send(Joint & joint)
{
  // Only what changed goes on the bus: a servo samples its pulse once a frame,
  // and the controller runs faster than that.
  const double target = std::clamp(joint.command, joint.lower, joint.upper);
  for (std::size_t k = 0; k < joint.servos.size(); ++k) {
    const ServoChannel & servo = joint.servos[k];
    const int counts = board_->counts_for(servo.pulse_us(target));
    if (counts != joint.sent[k]) {
      board_->set_counts(servo.channel, static_cast<uint16_t>(counts));
      joint.sent[k] = counts;
    }
  }
  joint.position = joint.servos[0].angle(joint.sent[0] * board_->count_us());
}

void Pca9685ServoSystem::stop()
{
  if (!board_) {
    return;
  }
  try {
    board_->all_off();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "could not switch the servos off: %s", error.what());
  }
  for (auto & joint : joints_) {
    std::fill(joint.sent.begin(), joint.sent.end(), -1);
  }
}

}  // namespace tattotronix_hardware

PLUGINLIB_EXPORT_CLASS(
  tattotronix_hardware::Pca9685ServoSystem, hardware_interface::SystemInterface)
