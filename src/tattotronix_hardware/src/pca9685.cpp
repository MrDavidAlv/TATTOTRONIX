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

#include "tattotronix_hardware/pca9685.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>

namespace tattotronix_hardware
{

uint8_t Pca9685::prescale_for(double frame_hz, double oscillator_hz)
{
  if (!(frame_hz > 0.0) || !(oscillator_hz > 0.0)) {
    throw std::invalid_argument("PCA9685 frame rate and oscillator must be positive");
  }
  const int64_t value = std::llround(oscillator_hz / (kCounts * frame_hz)) - 1;
  return static_cast<uint8_t>(std::clamp<int64_t>(value, 3, 255));
}

Pca9685::Pca9685(I2cBus & bus, double frame_hz, double oscillator_hz)
: bus_(bus), prescale_(prescale_for(frame_hz, oscillator_hz)), oscillator_hz_(oscillator_hz)
{
}

void Pca9685::start()
{
  bus_.write({kMode1, kSleep});
  bus_.write({kPrescale, prescale_});
  bus_.write({kMode2, kOutDrv});
  bus_.write({kMode1, kAutoIncrement});
  // The datasheet gives the oscillator 500 us to settle after it wakes.
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  all_off();
}

double Pca9685::count_us() const
{
  return (prescale_ + 1) / oscillator_hz_ * 1e6;
}

uint16_t Pca9685::counts_for(double pulse_us) const
{
  const double counts = std::round(pulse_us / count_us());
  return static_cast<uint16_t>(std::clamp(counts, 0.0, static_cast<double>(kCounts - 1)));
}

void Pca9685::set_counts(int channel, uint16_t off_count)
{
  if (channel < 0 || channel >= kChannels) {
    throw std::out_of_range("PCA9685 channel " + std::to_string(channel));
  }
  const uint16_t off = std::min<uint16_t>(off_count, kCounts - 1);
  bus_.write(
    {static_cast<uint8_t>(kLed0OnL + 4 * channel), 0x00, 0x00,
      static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>((off >> 8) & 0x0F)});
}

void Pca9685::set_off(int channel)
{
  if (channel < 0 || channel >= kChannels) {
    throw std::out_of_range("PCA9685 channel " + std::to_string(channel));
  }
  bus_.write({static_cast<uint8_t>(kLed0OnL + 4 * channel), 0x00, 0x00, 0x00, kFullOff});
}

void Pca9685::all_off()
{
  bus_.write({kAllLedOnL, 0x00, 0x00, 0x00, kFullOff});
}

}  // namespace tattotronix_hardware
