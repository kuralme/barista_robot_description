#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction

def generate_launch_description():
    # Setup variables
    robot_name1 = "rick"
    robot_name2 = "morty"
    position_r1 = [0.0, 0.0, 0.1]
    position_r2 = [3.0, 1.0, 0.1]
    orientation_r1 = [0.0, 0.0, 0.0]
    orientation_r2 = [0.0, 0.0, 0.0]

    xacro_file = "barista_robot_model.urdf.xacro"
    package_description = "barista_robot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), 'xacro', xacro_file)

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(package_description), 'meshes')
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    )

    # RVIZ Configuration - Delayed start
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'rick_morty.rviz')
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

    # Robot State Publishers
    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name1,
        parameters=[{'frame_prefix': robot_name1+'/', 'use_sim_time': True,
                    'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name1])}],
        output="screen"
    )
    rsp_robot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name2,
        parameters=[{'frame_prefix': robot_name2+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name2])}],
        output="screen"
    )

    # Spawn robots in Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity1',
        output='screen',
        arguments=[
            '-entity', robot_name1,
            '-topic', robot_name1+'/robot_description',
            '-x', str(position_r1[0]),
            '-y', str(position_r1[1]),
            '-z', str(position_r1[2]),
            '-R', str(orientation_r1[0]),
            '-P', str(orientation_r1[1]),
            '-Y', str(orientation_r1[2]),
        ]
    )
    spawn_robot2 = TimerAction(
        period=3.0,  # 3 sec delayed
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity2',
                output='screen',
                arguments=[
                    '-entity', robot_name2,
                    '-topic', robot_name2+'/robot_description',
                    '-x', str(position_r2[0]),
                    '-y', str(position_r2[1]),
                    '-z', str(position_r2[2]),
                    '-R', str(orientation_r2[0]),
                    '-P', str(orientation_r2[1]),
                    '-Y', str(orientation_r2[2]),
                ]
            )
        ]
    )
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp_robot1,
        rsp_robot2,
        spawn_robot1,
        spawn_robot2,
        delayed_rviz,
    ])
