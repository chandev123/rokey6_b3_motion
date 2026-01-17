#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[동작] 회피/삽입 모션 제어 노드

- 로봇 모션 명령 실행 (movel, movej)
- 순응 제어 (Compliance Control)
- 힘 기반 회피 동작
"""

import rclpy
from rclpy.node import Node
import DR_init
import numpy as np
from typing import List, Optional
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy


# Configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


class MotionControlNode(Node):
    """
    로봇 모션 제어 노드
    
    Subscribed Topics:
        ~/target_pose (Float64MultiArray): 목표 위치 [x,y,z,rx,ry,rz]
        /tactile_sensor/force_compensated: 힘 센서 피드백
    
    Services:
        ~/move_to_home: 홈 위치로 이동
        ~/move_to_approach: 접근 위치로 이동
        ~/execute_insertion: 삽입 동작 실행
        ~/execute_pull_test: Pull Test 실행
    
    Parameters:
        vel_fast, acc_fast: 빠른 이동 속도/가속도
        vel_slow, acc_slow: 삽입 시 속도/가속도
    """
    
    def __init__(self):
        super().__init__('motion_control')
        
        # Declare parameters
        self.declare_parameter('vel_fast', 100.0)
        self.declare_parameter('acc_fast', 100.0)
        self.declare_parameter('vel_slow', 30.0)
        self.declare_parameter('acc_slow', 30.0)
        self.declare_parameter('home_pos', [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.declare_parameter('approach_pos', [400.0, 0.0, 300.0, 180.0, 0.0, 180.0])
        
        self.vel_fast = self.get_parameter('vel_fast').value
        self.acc_fast = self.get_parameter('acc_fast').value
        self.vel_slow = self.get_parameter('vel_slow').value
        self.acc_slow = self.get_parameter('acc_slow').value
        self.home_pos = self.get_parameter('home_pos').value
        self.approach_pos = self.get_parameter('approach_pos').value
        
        # Import DSR_ROBOT2 after node setup
        from DSR_ROBOT2 import (
            movej, movel, posj, posx,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            get_current_posx, DR_BASE
        )
        
        self._movej = movej
        self._movel = movel
        self._posj = posj
        self._posx = posx
        self._get_current_posx = get_current_posx
        self._DR_BASE = DR_BASE
        
        # Set robot mode
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        
        # Current force feedback
        self.current_force = np.zeros(6)
        
        # Subscriber for force feedback
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.force_sub = self.create_subscription(
            Float64MultiArray,
            '/dsr01/tactile_sensor/force_compensated',
            self.force_callback,
            qos
        )
        
        # Services
        self.home_srv = self.create_service(
            Trigger, '~/move_to_home', self.move_to_home_callback)
        self.approach_srv = self.create_service(
            Trigger, '~/move_to_approach', self.move_to_approach_callback)
        self.insertion_srv = self.create_service(
            Trigger, '~/execute_insertion', self.execute_insertion_callback)
        self.pull_test_srv = self.create_service(
            Trigger, '~/execute_pull_test', self.execute_pull_test_callback)
        
        self.get_logger().info('MotionControlNode initialized')
    
    def force_callback(self, msg: Float64MultiArray):
        """힘 센서 피드백 수신"""
        self.current_force = np.array(msg.data)
    
    def move_to_home_callback(self, request, response):
        """홈 위치로 이동"""
        try:
            self.get_logger().info('Moving to home position...')
            self._movej(self._posj(*self.home_pos), 
                       vel=self.vel_fast, acc=self.acc_fast)
            response.success = True
            response.message = 'Moved to home position'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def move_to_approach_callback(self, request, response):
        """접근 위치로 이동"""
        try:
            self.get_logger().info('Moving to approach position...')
            self._movel(self._posx(*self.approach_pos),
                       vel=self.vel_fast, acc=self.acc_fast, ref=self._DR_BASE)
            response.success = True
            response.message = 'Moved to approach position'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def execute_insertion_callback(self, request, response):
        """삽입 동작 실행 (힘 기반 정지)"""
        try:
            self.get_logger().info('Executing insertion...')
            
            force_threshold = 10.0  # N
            max_steps = 100
            step_size = -1.0  # mm
            
            current_pos = list(self._get_current_posx()[0])
            
            for step in range(max_steps):
                # Check force
                fz = self.current_force[2]
                if fz > force_threshold:
                    self.get_logger().info(f'Contact detected! Fz={fz:.2f}N')
                    break
                
                # Move down
                current_pos[2] += step_size
                self._movel(self._posx(*current_pos),
                           vel=self.vel_slow, acc=self.acc_slow, ref=self._DR_BASE)
            
            response.success = True
            response.message = f'Insertion complete at Z={current_pos[2]:.1f}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def execute_pull_test_callback(self, request, response):
        """Pull Test 실행"""
        try:
            self.get_logger().info('Executing pull test...')
            
            pull_distance = 5.0  # mm
            current_pos = list(self._get_current_posx()[0])
            current_pos[2] += pull_distance
            
            self._movel(self._posx(*current_pos),
                       vel=self.vel_slow, acc=self.acc_slow, ref=self._DR_BASE)
            
            # Wait and measure
            import time
            time.sleep(0.1)
            
            pull_force = abs(self.current_force[2])
            
            response.success = True
            response.message = f'Pull force: {pull_force:.2f}N'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def move_linear(self, target_pos: List[float], fast: bool = True):
        """직선 이동"""
        vel = self.vel_fast if fast else self.vel_slow
        acc = self.acc_fast if fast else self.acc_slow
        self._movel(self._posx(*target_pos), vel=vel, acc=acc, ref=self._DR_BASE)
    
    def get_current_position(self) -> Optional[List[float]]:
        """현재 위치 반환"""
        try:
            return list(self._get_current_posx()[0])
        except:
            return None


def main(args=None):
    # Initialize DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    
    # Create base node for DR_init
    base_node = rclpy.create_node('motion_control_base', namespace=ROBOT_ID)
    DR_init.__dsr__node = base_node
    
    try:
        node = MotionControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        base_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
