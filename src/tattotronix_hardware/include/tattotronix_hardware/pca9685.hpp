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

#ifndef TATTOTRONIX_HARDWARE__PCA9685_HPP_
#define TATTOTRONIX_HARDWARE__PCA9685_HPP_

#include <cstdint>

#include "tattotronix_hardware/i2c_bus.hpp"

namespace tattotronix_hardware
{

/// The PCA9685 16-channel PWM chip, used as a servo pulse generator.
///
/// It divides each frame into 4096 counts of its prescaled oscillator. Every
/// channel here switches on at count 0 and off at a count set per channel, so a
/// pulse is always a whole number of counts: at 50 Hz one count is 4.88 us,
/// which is the command step docs/mathematical-model/actuators.md costs.
class Pca9685
{
public:
  static constexpr int kChannels = 16;
  static constexpr int kCounts = 4096;

  /// Registers and bits, from the NXP datasheet.
  static constexpr uint8_t kMode1 = 0x00;
  static constexpr uint8_t kMode2 = 0x01;
  static constexpr uint8_t kLed0OnL = 0x06;
  static constexpr uint8_t kAllLedOnL = 0xFA;
  static constexpr uint8_t kPrescale = 0xFE;
  static constexpr uint8_t kSleep = 0x10;
  static constexpr uint8_t kAutoIncrement = 0x20;
  static constexpr uint8_t kOutDrv = 0x04;
  static constexpr uint8_t kFullOff = 0x10;

  /// The prescaler for a frame rate: round(oscillator / (4096 * rate)) - 1,
  /// held to the 3 to 255 the chip accepts.
  static uint8_t prescale_for(double frame_hz, double oscillator_hz);

  Pca9685(I2cBus & bus, double frame_hz, double oscillator_hz);

  /// Set the frame rate and turn every channel off. The prescaler can only be
  /// written while the oscillator sleeps, so it sleeps, is set, and is woken
  /// with register auto-increment on and totem-pole outputs for the servos.
  void start();

  /// Microseconds per count at this prescaler.
  double count_us() const;

  /// The count a pulse of this width rounds to, held inside one frame.
  uint16_t counts_for(double pulse_us) const;

  /// Pulse a channel for `off_count` counts of every frame.
  void set_counts(int channel, uint16_t off_count);

  /// Stop pulsing a channel. A hobby servo with no pulse stops driving.
  void set_off(int channel);

  /// Stop every channel in one transaction.
  void all_off();

private:
  I2cBus & bus_;
  uint8_t prescale_;
  double oscillator_hz_;
};

}  // namespace tattotronix_hardware

#endif  // TATTOTRONIX_HARDWARE__PCA9685_HPP_
