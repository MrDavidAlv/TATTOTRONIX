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
Show the TATTOTRONIX description in RViz2, driven by joint sliders.

No simulator and no controllers: robot_state_publisher turns the xacro into
TF, joint_state_publisher_gui supplies the joint angles. This is the launch
file to reach for when the question is "is the kinematic tree right".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('tattotronix_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'tattotronix.urdf.xacro')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    tool = LaunchConfiguration('tool')
    use_rviz = LaunchConfiguration('use_rviz')

    # The description is evaluated at launch time so that 'tool' can be chosen
    # on the command line. ParameterValue(..., value_type=str) is what keeps
    # the resulting XML from being parsed as a YAML-ish scalar by rclcpp.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=none', ' tool:=', tool]),
        value_type=str,
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock instead of the wall clock',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Publish joint states from the slider GUI; '
                        'false uses the headless publisher',
        ),
        DeclareLaunchArgument(
            'tool',
            default_value='tattoo',
            description=(
                'End effector mounted on tool0: tattoo | none. The pen is sized '
                'from a catalogue rotary machine rather than measured; none '
                'strips the description back to the exported CAD'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the bundled configuration',
        ),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz2,
    ])
