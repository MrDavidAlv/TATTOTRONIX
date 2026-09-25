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
Draw on the arm, simulated or real, with one command.

    ros2 launch tattotronix_bringup draw.launch.py                      # Gazebo
    ros2 launch tattotronix_bringup draw.launch.py backend:=arm         # the real arm
    ros2 launch tattotronix_bringup draw.launch.py backend:=arm dry_run:=true
    ros2 launch tattotronix_bringup draw.launch.py art:=semillero speed:=2.0

It decides nothing itself. Each backend's package keeps its own entry point,
tattotronix_gazebo/draw.launch.py and tattotronix_hardware/arm.launch.py with
draw:=true, and this file includes the one `backend` names and hands it the
arguments. So what the real arm needs stays in tattotronix_hardware, where it
will be learnt, and this file does not have to change when it is.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BACKENDS = ('gazebo', 'arm')


def _include(context):
    backend = LaunchConfiguration('backend').perform(context)
    if backend not in BACKENDS:
        raise RuntimeError(
            f"backend:={backend} is not one of {', '.join(BACKENDS)}. "
            'gazebo draws in the simulator; arm draws on the real arm.')
    common = {'art': LaunchConfiguration('art'), 'speed': LaunchConfiguration('speed')}
    if backend == 'gazebo':
        path = os.path.join(get_package_share_directory('tattotronix_gazebo'),
                            'launch', 'draw.launch.py')
        args = dict(common, use_rviz=LaunchConfiguration('use_rviz'))
    else:
        path = os.path.join(get_package_share_directory('tattotronix_hardware'),
                            'launch', 'arm.launch.py')
        args = dict(common, draw='true', dry_run=LaunchConfiguration('dry_run'))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                     launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='gazebo',
            description='Where to draw: gazebo, the simulator, or arm, the real arm '
                        'through its PCA9685'),
        DeclareLaunchArgument(
            'art', default_value='ros_logo',
            description='Which drawing, by name under '
                        'tattotronix_control/config/trajectories'),
        DeclareLaunchArgument(
            'speed', default_value='1.0',
            description='Time scale for the drawing. 1.0 is the planned 6 mm/s '
                        'marking feed'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='backend:=gazebo only: open RViz with the ink trace'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='backend:=arm only: run the real driver against no board, '
                        'sending nothing'),
        OpaqueFunction(function=_include),
    ])
