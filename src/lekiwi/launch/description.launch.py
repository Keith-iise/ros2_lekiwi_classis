#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 路径配置
    pkg_path = FindPackageShare('lekiwi')
    urdf_path = PathJoinSubstitution([pkg_path, 'urdf', 'lekiwi.urdf'])
     # 读取机器人描述（标准写法，不会报错）
    robot_description = {
        'robot_description': Command(['xacro ', urdf_path])
    }

    # 机器人状态发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )


    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )


    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
    ])
