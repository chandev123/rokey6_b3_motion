#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DDR5 Tactile Handler Launch File

로봇 드라이버 + 3개 노드를 한 번에 실행:
    1. tactile_sensor     - 힘 센서 데이터 처리
    2. motion_control     - 로봇 모션 제어
    3. main_state_machine - 전체 순서 관리
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('ddr5_tactile_handler')
    config_file = os.path.join(pkg_share, 'config', 'ddr5_params.yaml')
    
    # Launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dsr01',
        description='Robot ID namespace'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start state machine'
    )
    
    # Robot namespace
    robot_id = LaunchConfiguration('robot_id')
    
    # =========================================================================
    # Node Definitions
    # =========================================================================
    
    # 1. Tactile Sensor Node (힘 센서)
    tactile_sensor_node = Node(
        package='ddr5_tactile_handler',
        executable='tactile_sensor',
        name='tactile_sensor',
        namespace=robot_id,
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # 2. Motion Control Node (모션 제어)
    motion_control_node = Node(
        package='ddr5_tactile_handler',
        executable='motion_control',
        name='motion_control',
        namespace=robot_id,
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # 3. Main State Machine Node (두뇌)
    main_state_machine_node = Node(
        package='ddr5_tactile_handler',
        executable='main_state_machine',
        name='main_state_machine',
        namespace=robot_id,
        parameters=[
            config_file,
            {'auto_start': LaunchConfiguration('auto_start')}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    
    return LaunchDescription([
        # Arguments
        robot_id_arg,
        auto_start_arg,
        
        # Nodes
        tactile_sensor_node,
        motion_control_node,
        main_state_machine_node,
    ])
