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

// Hold one servo at one pulse width, for calibration.
//
//   ros2 run tattotronix_hardware servo_pulse 0 1500
//   ros2 run tattotronix_hardware servo_pulse 0 1500 --device /dev/i2c-1 --address 64
//
// Drives a single PCA9685 channel with nothing else running: the way to find
// a servo's zero_us, the sign of its us_per_rad and its travel before the arm
// is driven as a whole (docs/hardware.md, section 5). New widths are read from
// standard input, one per line. An empty line or end of input stops, and
// stopping switches the channel off.

#include <exception>
#include <iostream>
#include <string>

#include "tattotronix_hardware/i2c_bus.hpp"
#include "tattotronix_hardware/pca9685.hpp"

namespace
{

// Wider than any hobby servo's travel on purpose: this is the tool for finding
// that travel. Anything outside it is a typing mistake.
constexpr double kLowestUs = 400.0;
constexpr double kHighestUs = 2600.0;

bool parse_width(const std::string & text, double & width)
{
  try {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || value < kLowestUs || value > kHighestUs) {
      return false;
    }
    width = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

int usage()
{
  std::cerr << "usage: servo_pulse CHANNEL WIDTH_US [--device /dev/i2c-1] [--address 64]\n"
            << "  CHANNEL is 0 to 15; WIDTH_US is " << kLowestUs << " to " << kHighestUs
            << " microseconds\n";
  return 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    return usage();
  }
  if ((argc - 3) % 2 != 0) {
    return usage();
  }
  std::string device = "/dev/i2c-1";
  int address = 0x40;
  int channel = -1;
  double width = 0.0;
  try {
    for (int i = 3; i + 1 < argc; i += 2) {
      const std::string flag = argv[i];
      if (flag == "--device") {
        device = argv[i + 1];
      } else if (flag == "--address") {
        address = std::stoi(argv[i + 1], nullptr, 0);
      } else {
        return usage();
      }
    }
    channel = std::stoi(argv[1]);
  } catch (const std::exception &) {
    return usage();
  }
  if (channel < 0 || channel >= tattotronix_hardware::Pca9685::kChannels ||
    address < 0 || address > 0x7F || !parse_width(argv[2], width))
  {
    return usage();
  }

  try {
    tattotronix_hardware::LinuxI2cBus bus(device, static_cast<uint8_t>(address));
    tattotronix_hardware::Pca9685 board(bus, 50.0, 25e6);
    board.start();
    std::string line = argv[2];
    do {
      if (!parse_width(line, width)) {
        std::cout << "not a width between " << kLowestUs << " and " << kHighestUs << " us\n";
        continue;
      }
      const uint16_t counts = board.counts_for(width);
      board.set_counts(channel, counts);
      std::cout << "channel " << channel << ": " << counts * board.count_us() << " us ("
                << counts << " counts). Another width, or an empty line to stop: "
                << std::flush;
    } while (std::getline(std::cin, line) && !line.empty());
    board.set_off(channel);
    std::cout << "\nchannel " << channel << " off\n";
  } catch (const std::exception & error) {
    std::cerr << "servo_pulse: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
