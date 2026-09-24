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
