#!/usr/bin/env python3
"""
simulation.launch.py
====================
Sobe a simulacao completa:
  1. Gazebo Harmonic com o mundo clinic.sdf
  2. robot_state_publisher (processa o xacro -> /robot_description e TF)
  3. spawn da cadeira no gz (a partir do topico /robot_description)
  4. ros_gz_bridge (config bridge.yaml)
  5. RViz2 (opcional)
  6. obstacle_mover (anima os obstaculos moveis)

Uso:
  ros2 launch wheelchair_gazebo simulation.launch.py
  ros2 launch wheelchair_gazebo simulation.launch.py rviz:=false
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = FindPackageShare('wheelchair_description')
    pkg_gz = FindPackageShare('wheelchair_gazebo')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'wheelchair.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg_gz, 'worlds', 'clinic.sdf'])
    bridge_cfg = PathJoinSubstitution([pkg_gz, 'config', 'bridge.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg_desc, 'rviz', 'wheelchair.rviz'])

    # ---------------- argumentos ----------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    use_safety = LaunchConfiguration('use_safety')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar o relogio da simulacao')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Abrir o RViz2')
    declare_use_safety = DeclareLaunchArgument(
        'use_safety', default_value='true',
        description='Rodar o arbitro de seguranca/desvio no circuito')

    # ---------------- 1) Gazebo ----------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------- 2) robot_state_publisher ----------------
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---------------- 3) spawn do robo ----------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wheelchair',
            '-x', '-4.5', '-y', '-1.5', '-z', '0.12', '-Y', '0.0',
        ],
    )

    # ---------------- 4) ponte ROS <-> gz ----------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': use_sim_time}],
    )

    # ---------------- 5) RViz ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    # ---------------- 6) animacao dos obstaculos moveis ----------------
    obstacle_mover = Node(
        package='wheelchair_control',
        executable='obstacle_mover',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------- 7) filtro do scan 360 (remove o proprio corpo) ----------------
    scan_filter = Node(
        package='wheelchair_control',
        executable='scan_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------------- 8) arbitro de seguranca / desvio ----------------
    # Fica NO CIRCUITO por padrao: teleop -> /cmd_vel_user -> arbitro -> /cmd_vel
    safety_arbiter = Node(
        package='wheelchair_control',
        executable='safety_arbiter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_safety),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_use_safety,
        gz_sim,
        rsp,
        spawn,
        bridge,
        rviz_node,
        obstacle_mover,
        scan_filter,
        safety_arbiter,
    ])
