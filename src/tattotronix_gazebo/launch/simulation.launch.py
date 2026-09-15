#!/usr/bin/env python3
"""
Bring the TATTOTRONIX arm up in Gazebo Sim (Fortress) under ros2_control.

Single source of truth for the simulated robot: the simulator, the description,
the spawn, the clock bridge and the controllers. Anything that wants a running
simulated arm includes this file rather than repeating the setup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _extend_gz_resource_path(*paths):
    """
    Put the install tree where Gazebo can find package:// mesh URIs.

    sdformat leaves package:// URIs untouched when it converts the URDF, and
    gz-common resolves them by looking for <package>/<rest> under each entry of
    the resource path. Pointing it at the directory that holds the package
    share folders is therefore what makes the meshes appear; without it the arm
    spawns as a collection of invisible links and the simulator says nothing.

    Fortress reads IGN_GAZEBO_RESOURCE_PATH. GZ_SIM_RESOURCE_PATH is set too so
    the same launch file keeps working on Garden and later.
    """
    for variable in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        current = os.environ.get(variable, '')
        entries = [p for p in current.split(':') if p]
        for path in paths:
            if path not in entries:
                entries.append(path)
        os.environ[variable] = ':'.join(entries)


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('tattotronix_gazebo')
    pkg_description = get_package_share_directory('tattotronix_description')
    pkg_control = get_package_share_directory('tattotronix_control')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_description, 'urdf', 'tattotronix.urdf.xacro')
    controllers_file = os.path.join(
        pkg_control, 'config', 'tattotronix_controllers.yaml')
    default_world = os.path.join(pkg_gazebo, 'worlds', 'studio.sdf')
    rviz_config_file = os.path.join(pkg_gazebo, 'rviz', 'simulation.rviz')

    _extend_gz_resource_path(
        os.path.dirname(pkg_description),
        os.path.join(pkg_gazebo, 'worlds'),
    )

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    tool = LaunchConfiguration('tool')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    spawn_z = LaunchConfiguration('spawn_z')

    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to the SDF world to load',
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
            description='Start RViz2 alongside the simulator',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Gazebo server without the GUI, for CI and tests',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='studio',
            description=(
                'Name of the world inside the SDF. Used to address the Gazebo '
                'control service; change it together with world'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.75',
            description=(
                'Height of the arm base. Matches the bench top in studio.sdf; '
                'change it together with the world'
            ),
        ),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware:=gz',
            ' tool:=', tool,
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r starts the world unpaused. A paused world never ticks, and a
            # controller manager that never ticks never finishes configuring,
            # so the spawners would time out waiting on a simulator that is
            # working exactly as told.
            'gz_args': ['-r -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v 2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    # Spawned from the topic rather than from a file so that the simulator and
    # robot_state_publisher are guaranteed to hold the same description.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_tattotronix',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tattotronix',
            '-x', '0.0',
            '-y', '0.0',
            '-z', spawn_z,
            '-Y', '0.0',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # Make sure the world is running before anything asks the controller
    # manager to switch controllers.
    #
    # gz_args already carries -r, but the GUI's WorldControl plugin publishes
    # its own run state at startup and can win that race, leaving the world
    # paused. A paused world does not tick, a controller manager that does not
    # tick never completes a switch, and the spawners fail on a five second
    # timeout that is not theirs to configure. The failure reads as a broken
    # controller when the simulator is simply standing still.
    unpause = ExecuteProcess(
        cmd=['ign', 'service',
             '-s', ['/world/', world_name, '/control'],
             '--reqtype', 'ignition.msgs.WorldControl',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '5000',
             '--req', 'pause: false'],
        output='screen',
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'controllers.launch.py')
        ),
    )

    # The controller manager only exists once the model carrying the plugin is
    # in the world, so the spawners wait for the spawn process to finish.
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[unpause])
    )

    controllers_after_unpause = RegisterEventHandler(
        OnProcessExit(target_action=unpause, on_exit=[controllers])
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(declared_arguments + [
        gz_sim,
        gz_sim_headless,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        controllers_after_spawn,
        controllers_after_unpause,
        rviz2,
    ])
