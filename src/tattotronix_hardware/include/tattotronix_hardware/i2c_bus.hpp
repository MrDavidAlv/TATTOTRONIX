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

#ifndef TATTOTRONIX_HARDWARE__I2C_BUS_HPP_
#define TATTOTRONIX_HARDWARE__I2C_BUS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace tattotronix_hardware
{

/// One device on an I2C bus, written one transaction at a time.
class I2cBus
{
public:
  virtual ~I2cBus() = default;

  /// Send the bytes to the device in a single transaction. Throws on failure.
  virtual void write(const std::vector<uint8_t> & bytes) = 0;
};

/// A device on a Linux i2c-dev bus, such as /dev/i2c-1 on a Raspberry Pi.
class LinuxI2cBus : public I2cBus
{
public:
  LinuxI2cBus(const std::string & device, uint8_t address);
  ~LinuxI2cBus() override;

  LinuxI2cBus(const LinuxI2cBus &) = delete;
  LinuxI2cBus & operator=(const LinuxI2cBus &) = delete;

  void write(const std::vector<uint8_t> & bytes) override;

private:
  int fd_;
  std::string device_;
};

/// Keeps every transaction instead of sending it: for tests, and for a dry run
/// of the whole stack on a machine with no board attached.
class RecordingBus : public I2cBus
{
public:
  void write(const std::vector<uint8_t> & bytes) override;

  std::vector<std::vector<uint8_t>> writes;
};

}  // namespace tattotronix_hardware

#endif  // TATTOTRONIX_HARDWARE__I2C_BUS_HPP_
