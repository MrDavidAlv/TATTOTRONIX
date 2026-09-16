#!/usr/bin/env python3
"""
Bring the arm up in Gazebo and draw the ROS logo.

Includes simulation.launch.py rather than repeating any of it, and starts
draw_logo once the controller spawners have exited. Starting it earlier only
means the action client waits, but waiting on an event that already exists is
cheaper than polling for one that does not.

    ros2 launch tattotronix_gazebo draw_logo.launch.py
    ros2 launch tattotronix_gazebo draw_logo.launch.py speed:=4.0

The ink is an RViz marker on /logo_trace: nothing in Gazebo leaves a mark when a
tool passes over a surface, so without it the arm moves and nothing appears.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
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
        executable='draw_logo',
        name='draw_logo',
        output='screen',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='1.0',
            description='Time scale for the trajectory. 1.0 is the planned '
                        '6 mm/s marking feed; anything faster is a preview, '
                        'not a result.'),
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
        # timer is honest about what it is: draw_logo waits for the action
        # server anyway, and this only keeps its log quiet until then.
        TimerAction(period=12.0, actions=[draw]),
    ])
