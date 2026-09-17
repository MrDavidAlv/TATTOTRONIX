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
Bring the arm up in Gazebo and draw the ROS logo.

Includes simulation.launch.py rather than repeating any of it, and starts
draw once the controller spawners have exited. Starting it earlier only
means the action client waits, but waiting on an event that already exists is
cheaper than polling for one that does not.

    ros2 launch tattotronix_gazebo draw.launch.py
    ros2 launch tattotronix_gazebo draw.launch.py art:=semillero
    ros2 launch tattotronix_gazebo draw.launch.py art:=foto speed:=2.0

`art` picks one of the trajectories installed by tattotronix_control; the node
lists what it has if the name is not among them.

The ink is an RViz marker on /ink_trace: nothing in Gazebo leaves a mark when a
tool passes over a surface, so without it the arm moves and nothing appears.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('tattotronix_gazebo')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items())

    draw = Node(
        package='tattotronix_control',
        executable='draw',
        name='draw',
        output='screen',
        parameters=[{
            'art': LaunchConfiguration('art'),
            'speed': LaunchConfiguration('speed'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'art', default_value='ros_logo',
            description='Which drawing to run. A name under '
                        'tattotronix_control/config/trajectories, without the '
                        '.npz. The node lists what is installed if the name '
                        'does not match.'),
        DeclareLaunchArgument(
            'speed', default_value='1.0',
            description='Time scale for the trajectory. 1.0 is the planned '
                        '6 mm/s marking feed. This rescales the motion itself, '
                        'not a playback rate: past roughly 2x the arm falls '
                        'outside the controller trajectory tolerance and the '
                        'goal is aborted mid-drawing.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='RViz is where the ink trace is visible.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_gazebo, 'rviz', 'simulation.rviz'),
            description='Passed through to simulation.launch.py.'),
        DeclareLaunchArgument(
            'settle', default_value='12.0',
            description='Seconds to wait for the controllers before sending.'),
        simulation,
        # The controllers are spawned by simulation.launch.py on its own event
        # chain, so there is no handle here to hang an OnProcessExit on. A
        # timer is honest about what it is: draw waits for the action
        # server anyway, and this only keeps its log quiet until then.
        TimerAction(period=12.0, actions=[draw]),
    ])
