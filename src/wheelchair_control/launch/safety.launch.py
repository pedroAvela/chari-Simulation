#!/usr/bin/env python3
"""
safety.launch.py
================
Sobe SOMENTE o arbitro de seguranca (autonomia compartilhada).

Use com o teclado, que roda em outro terminal:

  Terminal A:  ros2 launch wheelchair_control safety.launch.py
  Terminal B:  ros2 run wheelchair_control teleop_keyboard

Fluxo: teclado -> /cmd_vel_user -> safety_arbiter -> /cmd_vel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='wheelchair_control', executable='safety_arbiter',
            parameters=[{'use_sim_time': use_sim_time}], output='screen',
        ),
    ])
