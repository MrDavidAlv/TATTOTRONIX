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

#ifndef TATTOTRONIX_HARDWARE__PCA9685_SERVO_SYSTEM_HPP_
#define TATTOTRONIX_HARDWARE__PCA9685_SERVO_SYSTEM_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tattotronix_hardware/i2c_bus.hpp"
#include "tattotronix_hardware/pca9685.hpp"
#include "tattotronix_hardware/servo_channel.hpp"

namespace tattotronix_hardware
{

/// The TATTOTRONIX arm's hobby servos, driven through a PCA9685 board.
///
/// A hobby servo takes a position, closes its own loop and reports nothing
/// back. So the position this exports is the one last *sent* - after the joint
/// limits, the servo's pulse limits and the board's rounding - and the velocity
/// is the rate that changes at. It is an honest echo of the command, not a
/// measurement, and nothing downstream should read it as one.
class Pca9685ServoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685ServoSystem)

  /// Switches every channel off. The controller manager in Humble does not
  /// always deactivate its hardware when it is stopped, so the driver does not
  /// rely on it: a stopped arm is one whose servos have no pulse.
  ~Pca9685ServoSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// What a dry run would have sent to the board, or null when driving one.
  const RecordingBus * recording() const;

private:
  struct Joint
  {
    std::string name;
    std::vector<ServoChannel> servos;
    std::vector<int> sent;  // the count last sent to each servo; -1 before any
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    double command = 0.0;
    double position = 0.0;
    double previous = 0.0;
    double velocity = 0.0;
    bool has_velocity = false;
  };

  void send(Joint & joint);
  void stop();

  std::vector<Joint> joints_;
  std::string device_ = "/dev/i2c-1";
  int address_ = 0x40;
  double frame_hz_ = 50.0;
  double oscillator_hz_ = 25e6;
  bool dry_run_ = false;
  std::unique_ptr<I2cBus> bus_;
  std::unique_ptr<Pca9685> board_;
};

}  // namespace tattotronix_hardware

#endif  // TATTOTRONIX_HARDWARE__PCA9685_SERVO_SYSTEM_HPP_
