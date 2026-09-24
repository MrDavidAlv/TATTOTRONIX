cmake_minimum_required(VERSION 3.8)
project(tattotronix_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/i2c_bus.cpp
  src/pca9685.cpp
  src/servo_channel.cpp
  src/pca9685_servo_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface tattotronix_hardware.xml)

# Holds one servo at one pulse width, with nothing else running: how each servo
# is calibrated before the arm is driven as a whole (docs/hardware.md).
add_executable(servo_pulse src/servo_pulse.cpp)
target_link_libraries(servo_pulse ${PROJECT_NAME})
install(TARGETS servo_pulse DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pca9685 test/test_pca9685.cpp)
  target_link_libraries(test_pca9685 ${PROJECT_NAME})
  ament_add_gtest(test_pca9685_servo_system test/test_pca9685_servo_system.cpp)
  target_link_libraries(test_pca9685_servo_system ${PROJECT_NAME})

  # servo_pulse refuses a bad command line before it touches the bus, and says
  # so when the bus is not there. Neither needs a board to check.
  add_test(NAME servo_pulse_needs_its_arguments COMMAND servo_pulse)
  add_test(NAME servo_pulse_refuses_a_channel_the_board_lacks COMMAND servo_pulse 16 1500)
  add_test(NAME servo_pulse_refuses_a_width_no_servo_takes COMMAND servo_pulse 0 5000)
  add_test(NAME servo_pulse_says_when_there_is_no_bus
    COMMAND servo_pulse 0 1500 --device /dev/no-such-i2c-bus)
  # Each is held to what it says, not only to failing: a crash fails too.
  set_tests_properties(
    servo_pulse_needs_its_arguments
    servo_pulse_refuses_a_channel_the_board_lacks
    servo_pulse_refuses_a_width_no_servo_takes
    PROPERTIES PASS_REGULAR_EXPRESSION "usage: servo_pulse CHANNEL WIDTH_US")
  set_tests_properties(servo_pulse_says_when_there_is_no_bus
    PROPERTIES PASS_REGULAR_EXPRESSION "servo_pulse: cannot open /dev/no-such-i2c-bus")

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_files test/test_launch_files.py)

  # The whole bring-up, with the board simulated: ros2_control_node, the
  # plugin, the controllers and a trajectory.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_arm_dry_run.py TIMEOUT 240)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
