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
Start move_group against a robot that is already running.

This launches the planner and nothing else: no simulator, no controllers, no
robot_state_publisher. Those come from the simulation launch, and starting a
second copy of any of them is how two descriptions end up disagreeing about
where the arm is. Run the simulation first, then this.

Every parameter is loaded here explicitly rather than through a generator. The
file is longer that way and it is also readable, which matters more for the one
piece of configuration that decides whether the arm is allowed to move through
something.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(package, *parts):
    """Read a config file from a package's share directory."""
    path = os.path.join(get_package_share_directory(package), *parts)
    with open(path, 'r') as handle:
        return yaml.safe_load(handle)


def _text(package, *parts):
    """Read a config file verbatim. The SRDF goes in as a string."""
    path = os.path.join(get_package_share_directory(package), *parts)
    with open(path, 'r') as handle:
        return handle.read()


def generate_launch_description():
    """Build the launch description."""
    pkg_moveit = 'tattotronix_moveit_config'
    pkg_description = get_package_share_directory('tattotronix_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'tattotronix.urdf.xacro')

    hardware = LaunchConfiguration('hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'hardware',
            default_value='gz',
            choices=['gz', 'mock', 'pca9685'],
            description=(
                'Which hardware interface the description is built with. It '
                'must match the one the running system was started with, or '
                'the planner and the controller describe different robots'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description=(
                'Follow /clock. True whenever Gazebo is the one running; false '
                'with the real arm, hardware:=pca9685, which has no /clock'
            ),
        ),
    ]

    # The description is rebuilt here rather than read off the parameter server
    # so that move_group can start in any order relative to the rest.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=', hardware]),
        value_type=str,
    )

    # OMPL's own configuration is nested under the pipeline name. Humble reads
    # the pipeline list from these three keys, and silently plans with nothing
    # if the nesting is wrong, so it is spelled out.
    ompl = _yaml(pkg_moveit, 'config', 'ompl_planning.yaml')
    planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': _text(pkg_moveit, 'config', 'tattotronix.srdf')},
            {'robot_description_kinematics': _yaml(pkg_moveit, 'config', 'kinematics.yaml')},
            {'robot_description_planning': _yaml(pkg_moveit, 'config', 'joint_limits.yaml')},
            planning,
            _yaml(pkg_moveit, 'config', 'moveit_controllers.yaml'),
            {'use_sim_time': use_sim_time},
            # Without this move_group waits for a current state it will never
            # be told about and every plan fails on a timeout that looks like
            # a planner problem and is not.
            {'publish_robot_description_semantic': True},
        ],
    )

    return LaunchDescription(args + [move_group])
