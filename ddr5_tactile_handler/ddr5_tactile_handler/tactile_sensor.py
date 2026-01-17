#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[감각] 힘 센서 데이터 처리 노드

- 실시간 힘/토크 데이터 수신 및 전처리
- 중력 보상 (Gravity Compensation)
- 접촉 감지 이벤트 발행
"""

import rclpy
from rclpy.node import Node
import DR_init
import numpy as np
from collections import deque
from std_msgs.msg import Float64MultiArray, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy


# Configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ZEROING_DURATION = 1.0  # seconds
CONTROL_RATE = 100  # Hz


class TactileSensorNode(Node):
    """
    힘 센서 데이터 처리 노드
    
    Published Topics:
        ~/force_compensated (Float64MultiArray): 중력 보상된 힘 데이터 [Fx,Fy,Fz,Tx,Ty,Tz]
        ~/contact_detected (Bool): 접촉 감지 플래그
    
    Parameters:
        force_threshold (float): 접촉 감지 힘 임계값 (N)
    """
    
    def __init__(self):
        super().__init__('tactile_sensor')
        
        # Declare parameters
        self.declare_parameter('force_threshold', 5.0)
        self.declare_parameter('zeroing_duration', 1.0)
        
        self.force_threshold = self.get_parameter('force_threshold').value
        self.zeroing_duration = self.get_parameter('zeroing_duration').value
        
        # Import DSR_ROBOT2 after node setup
        from DSR_ROBOT2 import get_tool_force
        self._get_tool_force = get_tool_force
        
        # State
        self.gravity_bias = np.zeros(6)
        self.is_zeroed = False
        self.force_buffer = deque(maxlen=int(self.zeroing_duration * CONTROL_RATE))
        
        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.force_pub = self.create_publisher(
            Float64MultiArray, '~/force_compensated', qos)
        self.contact_pub = self.create_publisher(
            Bool, '~/contact_detected', qos)
        
        # Timer for sensor polling (100Hz)
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.timer_callback)
        
        self.get_logger().info('TactileSensorNode initialized')
        self.get_logger().info(f'Force threshold: {self.force_threshold} N')
    
    def timer_callback(self):
        """주기적 센서 데이터 처리"""
        try:
            raw_force = self._get_tool_force()
            if raw_force is None:
                return
            
            raw_force = np.array(raw_force)
            
            # Zeroing phase
            if not self.is_zeroed:
                self._do_zeroing(raw_force)
                return
            
            # Apply gravity compensation
            compensated_force = raw_force - self.gravity_bias
            
            # Publish compensated force
            msg = Float64MultiArray()
            msg.data = compensated_force.tolist()
            self.force_pub.publish(msg)
            
            # Check contact
            force_magnitude = np.linalg.norm(compensated_force[:3])
            contact_msg = Bool()
            contact_msg.data = force_magnitude > self.force_threshold
            self.contact_pub.publish(contact_msg)
            
        except Exception as e:
            self.get_logger().warning(f'Sensor read error: {e}')
    
    def _do_zeroing(self, raw_force: np.ndarray):
        """중력 보상을 위한 바이어스 계산"""
        self.force_buffer.append(raw_force)
        
        if len(self.force_buffer) >= self.force_buffer.maxlen:
            self.gravity_bias = np.mean(list(self.force_buffer), axis=0)
            self.is_zeroed = True
            self.get_logger().info(f'Zeroing complete. Bias: {self.gravity_bias}')
    
    def reset_zeroing(self):
        """외부에서 호출 가능한 제로잉 리셋"""
        self.is_zeroed = False
        self.force_buffer.clear()
        self.get_logger().info('Zeroing reset requested')


def main(args=None):
    # Initialize DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    
    # Create base node for DR_init
    base_node = rclpy.create_node('tactile_sensor_base', namespace=ROBOT_ID)
    DR_init.__dsr__node = base_node
    
    try:
        node = TactileSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        base_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
