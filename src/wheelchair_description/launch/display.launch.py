#!/usr/bin/env python3
"""
display.launch.py
Visualiza SOMENTE o modelo da cadeira no RViz, sem Gazebo.
Util para conferir a geometria/URDF e a arvore de TF.

  ros2 launch wheelchair_description display.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('wheelchair_description')

    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'wheelchair.urdf.xacro'])
    rviz_file = PathJoinSubstitution([pkg, 'rviz', 'wheelchair.rviz'])

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Abrir joint_state_publisher_gui para mover as rodas'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
    )

    return LaunchDescription([gui_arg, rsp, jsp_gui, rviz])
