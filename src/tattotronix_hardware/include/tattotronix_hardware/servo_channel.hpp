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

#ifndef TATTOTRONIX_HARDWARE__SERVO_CHANNEL_HPP_
#define TATTOTRONIX_HARDWARE__SERVO_CHANNEL_HPP_

namespace tattotronix_hardware
{

/// How one hobby servo's pulse width maps to the angle of the joint it turns.
///
/// Every value is a calibration of one servo on one arm. The defaults in
/// tattotronix_description/config/servo_calibration.yaml are the catalogue
/// convention, not a measurement.
struct ServoChannel
{
  /// PCA9685 output, 0 to 15.
  int channel = -1;
  /// The pulse at joint angle zero, us.
  double zero_us = 1500.0;
  /// Pulse per radian of joint angle. Signed: the sign is the way round the
  /// servo is mounted, and two servos on one joint usually differ in it.
  double us_per_rad = 636.62;
  /// The pulse is never sent outside these, whatever the joint is asked for.
  double min_us = 500.0;
  double max_us = 2500.0;

  /// The pulse for a joint angle, held to [min_us, max_us].
  double pulse_us(double angle) const;

  /// The joint angle a pulse stands for.
  double angle(double pulse_us) const;

  /// Empty when the calibration can be used; otherwise what is wrong with it.
  const char * problem() const;
};

}  // namespace tattotronix_hardware

#endif  // TATTOTRONIX_HARDWARE__SERVO_CHANNEL_HPP_
