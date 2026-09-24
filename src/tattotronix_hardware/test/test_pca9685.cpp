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

// The PCA9685 at the register level, and the servo calibration. The expected
// bytes are the NXP datasheet's, written out, so a change to the driver that
// still compiles but talks to the chip differently fails here.

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "tattotronix_hardware/i2c_bus.hpp"
#include "tattotronix_hardware/pca9685.hpp"
#include "tattotronix_hardware/servo_channel.hpp"

using tattotronix_hardware::Pca9685;
using tattotronix_hardware::RecordingBus;
using tattotronix_hardware::ServoChannel;
using Bytes = std::vector<uint8_t>;

TEST(Pca9685, PrescalerIsTheDatasheetFormula)
{
  // round(25 MHz / (4096 * 50 Hz)) - 1 = round(122.07) - 1
  EXPECT_EQ(Pca9685::prescale_for(50.0, 25e6), 121);
  EXPECT_EQ(Pca9685::prescale_for(60.0, 25e6), 101);
  // The chip will not go outside 3 to 255.
  EXPECT_EQ(Pca9685::prescale_for(5000.0, 25e6), 3);
  EXPECT_EQ(Pca9685::prescale_for(1.0, 25e6), 255);
  EXPECT_THROW(Pca9685::prescale_for(0.0, 25e6), std::invalid_argument);
}

TEST(Pca9685, OneCountAtFiftyHertzIsTheStepTheActuatorStudyCosts)
{
  RecordingBus bus;
  Pca9685 board(bus, 50.0, 25e6);
  // 122 oscillator periods: 4.88 us, a 20 ms frame in 4096 counts to within the
  // prescaler's rounding.
  EXPECT_NEAR(board.count_us(), 4.88, 1e-9);
  EXPECT_EQ(board.counts_for(1500.0), 307);
  EXPECT_EQ(board.counts_for(1e6), 4095);
  EXPECT_EQ(board.counts_for(-10.0), 0);
}

TEST(Pca9685, StartSetsTheFrameRateAsleepThenWakesWithEverythingOff)
{
  RecordingBus bus;
  Pca9685 board(bus, 50.0, 25e6);
  board.start();
  const std::vector<Bytes> expected{
    {0x00, 0x10},                    // MODE1: sleep, so the prescaler can be written
    {0xFE, 121},                     // PRESCALE
    {0x01, 0x04},                    // MODE2: totem-pole outputs
    {0x00, 0x20},                    // MODE1: awake, register auto-increment
    {0xFA, 0x00, 0x00, 0x00, 0x10},  // ALL_LED: full off
  };
  EXPECT_EQ(bus.writes, expected);
}

TEST(Pca9685, AChannelIsOnAtZeroAndOffAtItsCount)
{
  RecordingBus bus;
  Pca9685 board(bus, 50.0, 25e6);
  board.set_counts(3, 307);  // 307 = 0x133
  board.set_off(15);
  const std::vector<Bytes> expected{
    {0x06 + 4 * 3, 0x00, 0x00, 0x33, 0x01},
    {0x06 + 4 * 15, 0x00, 0x00, 0x00, 0x10},
  };
  EXPECT_EQ(bus.writes, expected);
  EXPECT_THROW(board.set_counts(16, 307), std::out_of_range);
  EXPECT_THROW(board.set_off(-1), std::out_of_range);
}

TEST(ServoChannel, PulseAndAngleAreInversesInsideTheLimits)
{
  ServoChannel servo;
  servo.channel = 0;
  servo.zero_us = 1450.0;
  servo.us_per_rad = -600.0;
  EXPECT_DOUBLE_EQ(servo.pulse_us(0.0), 1450.0);
  EXPECT_DOUBLE_EQ(servo.pulse_us(0.5), 1150.0);  // the sign is the mounting
  EXPECT_DOUBLE_EQ(servo.angle(servo.pulse_us(0.3)), 0.3);
  EXPECT_DOUBLE_EQ(servo.pulse_us(-10.0), servo.max_us);
  EXPECT_DOUBLE_EQ(servo.pulse_us(10.0), servo.min_us);
  EXPECT_STREQ(servo.problem(), "");
}

TEST(ServoChannel, AnUnusableCalibrationSaysWhy)
{
  ServoChannel servo;
  EXPECT_STRNE(servo.problem(), "");  // no channel
  servo.channel = 2;
  servo.us_per_rad = 0.0;
  EXPECT_STRNE(servo.problem(), "");
  servo.us_per_rad = 636.62;
  servo.min_us = 2600.0;
  EXPECT_STRNE(servo.problem(), "");
  servo.min_us = 500.0;
  servo.zero_us = 3000.0;
  EXPECT_STRNE(servo.problem(), "");
  servo.zero_us = 1500.0;
  EXPECT_STREQ(servo.problem(), "");
}
