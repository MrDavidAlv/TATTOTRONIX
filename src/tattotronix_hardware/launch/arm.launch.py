#!/usr/bin/env python3
# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Bring the real TATTOTRONIX arm up: its servos, through a PCA9685, under ros2_control.

The controllers are the simulation's, from the same file, so what the arm is
asked to do cannot drift between the two. config/hardware.yaml changes only what
has to: there is no simulator clock, and the controller manager runs at 100 Hz,
because a hobby servo samples its pulse once every 20 ms.

    ros2 launch tattotronix_hardware arm.launch.py dry_run:=true   # sends nothing
    ros2 launch tattotronix_hardware arm.launch.py                 # the arm moves
    ros2 launch tattotronix_hardware arm.launch.py draw:=true art:=ros_logo

When the controllers come up, every servo is sent to the initial pose, every
joint at zero, and goes there at full speed. Put the arm near it by hand first.
docs/hardware.md has the whole procedure, calibration first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_hardware = get_package_share_directory('tattotronix_hardware')
    pkg_description = get_package_share_directory('tattotronix_description')
    pkg_control = get_package_share_directory('tattotronix_control')

    xacro_file = os.path.join(pkg_description, 'urdf', 'tattotronix.urdf.xacro')
    controllers_file = os.path.join(pkg_control, 'config', 'tattotronix_controllers.yaml')
    hardware_file = os.path.join(pkg_hardware, 'config', 'hardware.yaml')

    arguments = [
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true runs everything with the PCA9685 simulated, sending '
                        'nothing: the way to check a setup before any servo is '
                        'powered'),
        DeclareLaunchArgument(
            'i2c_device', default_value='/dev/i2c-1',
            description='The Linux I2C bus the PCA9685 is on. On a Raspberry Pi, '
                        'bus 1 is the one on pins 3 and 5'),
        DeclareLaunchArgument(
            'i2c_address', default_value='64',
            description='The PCA9685 address. 64 is 0x40, a board with no address '
                        'jumpers soldered'),
        DeclareLaunchArgument(
            'servo_calibration',
            default_value=os.path.join(pkg_description, 'config', 'servo_calibration.yaml'),
            description='Which channel drives each joint, and how its pulse maps to '
                        'the angle. The default is the catalogue convention, not a '
                        'measurement'),
        DeclareLaunchArgument(
            'draw', default_value='false',
            description='Also start the draw node, which sends the drawing once '
                        'the arm controller answers'),
        DeclareLaunchArgument(
            'art', default_value='ros_logo',
            description='Which drawing to send with draw:=true, by name under '
                        'tattotronix_control/config/trajectories'),
        DeclareLaunchArgument(
            'speed', default_value='1.0',
            description='Time scale for the drawing. 1.0 is the planned 6 mm/s '
                        'marking feed'),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware:=pca9685',
            ' dry_run:=', LaunchConfiguration('dry_run'),
            ' i2c_device:=', LaunchConfiguration('i2c_device'),
            ' i2c_address:=', LaunchConfiguration('i2c_address'),
            ' servo_calibration:=', LaunchConfiguration('servo_calibration'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # The overrides come after the shared file, so where both name a parameter
    # the hardware's value wins.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_file, hardware_file],
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'controllers.launch.py')),
        launch_arguments={'spawn_timeout': '30'}.items(),
    )

    # The node waits for the controller's action server itself, so it can start
    # with everything else.
    draw = Node(
        package='tattotronix_control',
        executable='draw',
        name='draw',
        output='screen',
        parameters=[{
            'art': LaunchConfiguration('art'),
            'speed': LaunchConfiguration('speed'),
            'use_sim_time': False,
        }],
        condition=IfCondition(LaunchConfiguration('draw')),
    )

    return LaunchDescription(
        arguments + [robot_state_publisher, control_node, controllers, draw])
