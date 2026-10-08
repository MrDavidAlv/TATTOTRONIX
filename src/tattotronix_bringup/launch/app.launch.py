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
The arm, simulated or real, waiting for the tablet app.

    ros2 launch tattotronix_bringup app.launch.py                  # Gazebo
    ros2 launch tattotronix_bringup app.launch.py backend:=arm     # the real arm

Starts the backend without the drawing - the app decides what to draw - the
draw server, whose draw_strokes action draws the strokes the app sends, and
rosbridge, which serves the arm's topics, services and actions to the app as
JSON over a WebSocket. Over Wi-Fi the app connects to ws://<robot-ip>:<port>;
over USB, `adb reverse tcp:9090 tcp:9090` on the workstation lets it use
ws://127.0.0.1:9090.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BACKENDS = ('gazebo', 'arm')


def _include(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in BACKENDS:
        raise RuntimeError(
            f"backend:={backend} is not one of {', '.join(BACKENDS)}. "
            'gazebo runs the simulator; arm runs the real arm.')
    if backend == 'gazebo':
        path = os.path.join(get_package_share_directory('tattotronix_gazebo'),
                            'launch', 'simulation.launch.py')
        args = {'use_rviz': LaunchConfiguration('use_rviz'),
                'headless': LaunchConfiguration('headless')}
    else:
        path = os.path.join(get_package_share_directory('tattotronix_hardware'),
                            'launch', 'arm.launch.py')
        args = {'draw': 'false', 'dry_run': LaunchConfiguration('dry_run')}
    rosbridge = os.path.join(get_package_share_directory('rosbridge_server'),
                             'launch', 'rosbridge_websocket_launch.xml')
    # A drawing is an action that runs for minutes. Without its own thread,
    # rosbridge 2.0 waits on it and answers nothing else meanwhile: the app's
    # joint states, its cancel. Service calls likewise, and they time out.
    bridge_args = {
        'port': LaunchConfiguration('port'),
        'send_action_goals_in_new_thread': 'true',
        'call_services_in_new_thread': 'true',
        'default_call_service_timeout': '5.0',
    }
    draw_server = Node(
        package='tattotronix_control',
        executable='draw_server',
        name='draw_server',
        output='screen',
        parameters=[{'use_sim_time': backend == 'gazebo'}],
    )
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                 launch_arguments=args.items()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge),
                                 launch_arguments=bridge_args.items()),
        draw_server,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='gazebo',
            description='Which arm the app drives: gazebo, the simulator, or arm, '
                        'the real arm through its PCA9685'),
        DeclareLaunchArgument(
            'port', default_value='9090',
            description='The rosbridge WebSocket port the app connects to'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='backend:=gazebo only: also open RViz. Off by default, the '
                        'app shows the arm itself'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='backend:=gazebo only: run the simulator without its window'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='backend:=arm only: run the real driver against no board, '
                        'sending nothing'),
        OpaqueFunction(function=_include),
    ])
