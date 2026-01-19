#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project H.I.T - Robot Control Launch File
-----------------------------------------
이 launch 파일은 다음 노드들을 실행합니다:
1. robot_interface_node - 로봇 모션 제어 및 상태 퍼블리싱
2. gripper_service_node - 그리퍼 제어 서비스
3. task_controller_node - 작업 제어 (선택사항)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # Launch Arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value='dsr01',
            description='Robot ID (namespace)'
        ),
        
        LogInfo(msg='Starting Project H.I.T Robot Control...'),
        
        # 1. Robot Interface Node (로봇 제어 및 상태 모니터링)
        Node(
            package='project_hit',
            executable='robot_interface_node',
            name='robot_interface_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # 2. Gripper Service Node (그리퍼 제어)
        Node(
            package='project_hit',
            executable='gripper_service_node',
            name='gripper_service_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # 3. Task Controller Node (작업 제어 - 선택사항)
        # 주석 해제하면 자동 작업 제어 활성화
        # Node(
        #     package='project_hit',
        #     executable='task_controller_node',
        #     name='task_controller_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
        
        LogInfo(msg='All nodes started successfully!'),
    ])
