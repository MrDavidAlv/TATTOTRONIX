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

#include "tattotronix_hardware/i2c_bus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <string>
#include <system_error>
#include <vector>

namespace tattotronix_hardware
{

LinuxI2cBus::LinuxI2cBus(const std::string & device, uint8_t address)
: fd_(-1), device_(device)
{
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    // The usual cause on a Raspberry Pi is I2C not enabled, or a user outside
    // the i2c group. Both are named in docs/hardware.md.
    throw std::system_error(errno, std::generic_category(), "cannot open " + device);
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(
            error, std::generic_category(),
            "cannot select device " + std::to_string(address) + " on " + device);
  }
}

LinuxI2cBus::~LinuxI2cBus()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void LinuxI2cBus::write(const std::vector<uint8_t> & bytes)
{
  const ssize_t sent = ::write(fd_, bytes.data(), bytes.size());
  if (sent != static_cast<ssize_t>(bytes.size())) {
    throw std::system_error(errno, std::generic_category(), "I2C write on " + device_);
  }
}

void RecordingBus::write(const std::vector<uint8_t> & bytes)
{
  writes.push_back(bytes);
}

}  // namespace tattotronix_hardware
