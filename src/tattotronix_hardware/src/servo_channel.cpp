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

#include "tattotronix_hardware/servo_channel.hpp"

#include <algorithm>
#include <cmath>

#include "tattotronix_hardware/pca9685.hpp"

namespace tattotronix_hardware
{

double ServoChannel::pulse_us(double angle) const
{
  return std::clamp(zero_us + us_per_rad * angle, min_us, max_us);
}

double ServoChannel::angle(double pulse) const
{
  return (pulse - zero_us) / us_per_rad;
}

const char * ServoChannel::problem() const
{
  if (channel < 0 || channel >= Pca9685::kChannels) {
    return "channel must be 0 to 15";
  }
  if (!std::isfinite(us_per_rad) || us_per_rad == 0.0) {
    return "us_per_rad must be finite and not zero";
  }
  if (!(min_us > 0.0) || !(min_us < max_us)) {
    return "min_us must be positive and below max_us";
  }
  if (!(zero_us >= min_us && zero_us <= max_us)) {
    return "zero_us must lie between min_us and max_us";
  }
  return "";
}

}  // namespace tattotronix_hardware
