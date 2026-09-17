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
Activate the TATTOTRONIX controllers on an already running controller manager.

This file does not start the controller manager. In simulation the Gazebo Sim
plugin owns it; on hardware ros2_control_node will. All that happens here is
the ordered spawning of the controllers, which is a separate concern precisely
because the same order applies to both.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_manager = LaunchConfiguration('controller_manager')
    spawn_timeout = LaunchConfiguration('spawn_timeout')

    declared_arguments = [
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
            description='Node name of the controller manager to spawn into',
        ),
        DeclareLaunchArgument(
            'spawn_timeout',
            default_value='60',
            description=(
                'Seconds the spawner waits for the controller manager. Gazebo Sim '
                'loads meshes before it configures the plugin, so the manager can '
                'take a while to answer on a cold start'
            ),
        ),
    ]

    # The broadcaster goes first and on its own. Until it is active nothing
    # publishes /joint_states, so a trajectory controller spawned alongside it
    # would come up against a robot whose state is unknown.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', spawn_timeout,
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', spawn_timeout,
        ],
    )

    arm_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription(declared_arguments + [
        joint_state_broadcaster_spawner,
        arm_controller_after_broadcaster,
    ])
