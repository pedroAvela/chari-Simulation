#!/usr/bin/env python3
"""
teleop_joystick.launch.py
=========================
Sobe a pilha de controle por JOYSTICK com autonomia compartilhada:

  joy_node -> /joy -> joystick_teleop -> /cmd_vel_user -> safety_arbiter -> /cmd_vel

Uso (com a simulacao ja rodando):
  ros2 launch wheelchair_control teleop_joystick.launch.py

Para desligar o arbitro (joystick vai direto ao robo em /cmd_vel):
  ros2 launch wheelchair_control teleop_joystick.launch.py use_safety:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('wheelchair_control')
    joy_cfg = PathJoinSubstitution([pkg, 'config', 'joystick.yaml'])

    use_safety = LaunchConfiguration('use_safety')
    use_sim_time = LaunchConfiguration('use_sim_time')

    decl = [
        DeclareLaunchArgument('use_safety', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen',
    )

    # --- COM arbitro: joystick -> /cmd_vel_user -> arbitro -> /cmd_vel ---
    with_safety = GroupAction(
        condition=IfCondition(use_safety),
        actions=[
            Node(package='wheelchair_control', executable='joystick_teleop',
                 parameters=[joy_cfg, {'use_sim_time': use_sim_time}],
                 output='screen'),
            Node(package='wheelchair_control', executable='safety_arbiter',
                 parameters=[{'use_sim_time': use_sim_time}],
                 output='screen'),
        ],
    )

    # --- SEM arbitro: joystick -> /cmd_vel (direto) ---
    without_safety = GroupAction(
        condition=UnlessCondition(use_safety),
        actions=[
            Node(package='wheelchair_control', executable='joystick_teleop',
                 parameters=[joy_cfg, {'output_topic': '/cmd_vel'},
                             {'use_sim_time': use_sim_time}],
                 output='screen'),
        ],
    )

    return LaunchDescription(decl + [joy_node, with_safety, without_safety])
