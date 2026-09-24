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

// The ros2_control side of the driver, in a dry run: what it accepts, what it
// refuses, and what it would send to the board.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tattotronix_hardware/pca9685_servo_system.hpp"

using tattotronix_hardware::Pca9685ServoSystem;
using CallbackReturn = Pca9685ServoSystem::CallbackReturn;

namespace
{

// joint_1 on one servo, joint_2 on two mounted opposite ways, as the arm's
// shoulder is.
std::string urdf(
  const std::string & joint_1_extra = "", const std::string & states = "",
  const std::string & tool = "")
{
  return
    R"(<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <ros2_control name="arm" type="system">
    <hardware>
      <plugin>tattotronix_hardware/Pca9685ServoSystem</plugin>
      <param name="dry_run">true</param>
    </hardware>
    <joint name="joint_1">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.2</param>
      </state_interface>
      <state_interface name="velocity"/>)" + states +
    R"(
      <param name="channel">0</param>
      <param name="zero_us">1500</param>
      <param name="us_per_rad">636.62</param>)" + joint_1_extra +
    R"(
    </joint>
    <joint name="joint_2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="channel">1</param>
      <param name="us_per_rad">636.62</param>
      <param name="channel_b">2</param>
      <param name="us_per_rad_b">-636.62</param>
    </joint>)" + tool +
    R"(
  </ros2_control>
</robot>)";
}

hardware_interface::HardwareInfo info_of(const std::string & description)
{
  return hardware_interface::parse_control_resources_from_urdf(description).at(0);
}

double value_of(
  const std::vector<hardware_interface::StateInterface> & states, const std::string & name)
{
  for (const auto & state : states) {
    if (state.get_name() == name) {
      return state.get_value();
    }
  }
  ADD_FAILURE() << "no state interface " << name;
  return std::nan("");
}

// Half a count of the board, in joint angle: the most rounding can move it.
constexpr double kHalfCount = 0.5 * 4.88 / 636.62;

}  // namespace

TEST(Pca9685ServoSystem, LoadsAsAPluginTheWayTheControllerManagerDoes)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto system = loader.createSharedInstance("tattotronix_hardware/Pca9685ServoSystem");
  ASSERT_NE(system, nullptr);
  EXPECT_EQ(system->on_init(info_of(urdf())), CallbackReturn::SUCCESS);
}

TEST(Pca9685ServoSystem, HoldsTheInitialPoseThenFollowsItsCommands)
{
  Pca9685ServoSystem system;
  ASSERT_EQ(system.on_init(info_of(urdf())), CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(states.size(), 3u);  // joint_1 position and velocity, joint_2 position
  ASSERT_EQ(commands.size(), 2u);

  const rclcpp_lifecycle::State unused;
  ASSERT_EQ(system.on_configure(unused), CallbackReturn::SUCCESS);
  ASSERT_NE(system.recording(), nullptr);
  ASSERT_EQ(system.on_activate(unused), CallbackReturn::SUCCESS);
  EXPECT_NEAR(value_of(states, "joint_1/position"), 0.2, kHalfCount);

  const rclcpp::Time now(0, 0);
  const auto period = rclcpp::Duration::from_seconds(0.01);
  commands[0].set_value(0.5);
  commands[1].set_value(0.3);
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  ASSERT_EQ(system.read(now, period), hardware_interface::return_type::OK);
  EXPECT_NEAR(value_of(states, "joint_1/position"), 0.5, kHalfCount);
  EXPECT_NEAR(value_of(states, "joint_1/velocity"), (0.5 - 0.2) / 0.01, 2 * kHalfCount / 0.01);
  EXPECT_NEAR(value_of(states, "joint_2/position"), 0.3, kHalfCount);

  // The shoulder's two servos get mirrored pulses: 1500 us plus and minus the
  // same offset, rounded to the board's counts.
  const auto & writes = system.recording()->writes;
  auto last_count = [&writes](int channel) {
      int count = -1;
      for (const auto & w : writes) {
        if (w.size() == 5 && w[0] == 0x06 + 4 * channel) {
          count = w[3] | (w[4] << 8);
        }
      }
      return count;
    };
  EXPECT_EQ(last_count(1), static_cast<int>(std::round((1500 + 636.62 * 0.3) / 4.88)));
  EXPECT_EQ(last_count(2), static_cast<int>(std::round((1500 - 636.62 * 0.3) / 4.88)));

  // An unchanged command puts nothing on the bus.
  const auto sent = writes.size();
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_EQ(writes.size(), sent);
}

TEST(Pca9685ServoSystem, NeverSendsPastTheJointLimits)
{
  Pca9685ServoSystem system;
  ASSERT_EQ(system.on_init(info_of(urdf())), CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  const rclcpp_lifecycle::State unused;
  ASSERT_EQ(system.on_configure(unused), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(unused), CallbackReturn::SUCCESS);
  commands[0].set_value(3.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0, 0), rclcpp::Duration::from_seconds(0.01)),
    hardware_interface::return_type::OK);
  EXPECT_NEAR(value_of(states, "joint_1/position"), 1.57, kHalfCount);
}

TEST(Pca9685ServoSystem, DeactivatingSwitchesEveryServoOff)
{
  Pca9685ServoSystem system;
  ASSERT_EQ(system.on_init(info_of(urdf())), CallbackReturn::SUCCESS);
  const rclcpp_lifecycle::State unused;
  ASSERT_EQ(system.on_configure(unused), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(unused), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_deactivate(unused), CallbackReturn::SUCCESS);
  const std::vector<uint8_t> all_off{0xFA, 0x00, 0x00, 0x00, 0x10};
  EXPECT_EQ(system.recording()->writes.back(), all_off);
}

TEST(Pca9685ServoSystem, RefusesWhatItCannotDo)
{
  const std::vector<std::string> bad_extras{
    "<param name=\"channel_b\">1</param>",   // a channel already in use
    "<param name=\"min_us\">3000</param>",   // limits the wrong way round
    "<param name=\"max_us\">25OO</param>",   // not a number
  };
  for (const auto & extra : bad_extras) {
    Pca9685ServoSystem system;
    EXPECT_EQ(system.on_init(info_of(urdf(extra))), CallbackReturn::ERROR) << extra;
  }
  // A hobby servo reports no effort, so asking for one is an error, not a zero.
  Pca9685ServoSystem system;
  EXPECT_EQ(
    system.on_init(info_of(urdf("", "\n      <state_interface name=\"effort\"/>"))),
    CallbackReturn::ERROR);
}

namespace
{

// The tool's continuous servo on channel 6: stops at 1500 us, full speed 500 us
// either side, as the declared calibration has it.
std::string tool(const std::string & channel = "6")
{
  return
    R"(
    <gpio name="tool">
      <command_interface name="speed"/>
      <state_interface name="speed"/>
      <param name="channel">)" + channel +
    R"(</param>
      <param name="stop_us">1500</param>
      <param name="us_per_speed">500</param>
      <param name="min_us">1000</param>
      <param name="max_us">2000</param>
    </gpio>)";
}

}  // namespace

TEST(Pca9685ServoSystem, TheToolTurnsOnlyWhenAskedAndNeverPastFullSpeed)
{
  Pca9685ServoSystem system;
  ASSERT_EQ(system.on_init(info_of(urdf("", "", tool()))), CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(commands.size(), 3u);  // two joints and the tool
  const rclcpp_lifecycle::State unused;
  ASSERT_EQ(system.on_configure(unused), CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(unused), CallbackReturn::SUCCESS);

  // Activated, the tool's channel is off: no pulse, whatever its trim.
  const std::vector<uint8_t> off{0x06 + 4 * 6, 0x00, 0x00, 0x00, 0x10};
  EXPECT_EQ(system.recording()->writes.back(), off);
  EXPECT_EQ(value_of(states, "tool/speed"), 0.0);

  const rclcpp::Time now(0, 0);
  const auto period = rclcpp::Duration::from_seconds(0.01);
  const double half_count = 0.5 * 4.88 / 500.0;
  commands[2].set_value(0.5);
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  const auto & sent = system.recording()->writes.back();
  EXPECT_EQ(sent[0], 0x06 + 4 * 6);
  EXPECT_EQ(sent[3] | (sent[4] << 8), static_cast<int>(std::round((1500 + 250) / 4.88)));
  EXPECT_NEAR(value_of(states, "tool/speed"), 0.5, half_count);

  commands[2].set_value(3.0);  // past full speed
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_NEAR(value_of(states, "tool/speed"), 1.0, half_count);

  commands[2].set_value(0.0);
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_EQ(system.recording()->writes.back(), off);
  EXPECT_EQ(value_of(states, "tool/speed"), 0.0);
}

TEST(Pca9685ServoSystem, TheToolNeedsAChannelOfItsOwn)
{
  Pca9685ServoSystem system;
  EXPECT_EQ(system.on_init(info_of(urdf("", "", tool("0")))), CallbackReturn::ERROR);
}
