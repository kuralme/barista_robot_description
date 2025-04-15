#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction

def generate_launch_description():
    # Setup variables
    robot_name = "morty"
    position = [0.0, 0.0, 0.1]
    orientation = [0.0, 0.0, 0.0]

    urdf_file = 'barista_robot_model.urdf'
    package_description = "barista_robot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(package_description), 'meshes')
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
    )
    delayed_rviz = TimerAction(
        actions=[rviz_node],
        period=5.0
    )
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', robot_desc_path])
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2]),
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        delayed_rviz,
    ])
