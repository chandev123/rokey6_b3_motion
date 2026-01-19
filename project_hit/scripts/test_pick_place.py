#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project H.I.T - Simple Pick and Place Test Script
-------------------------------------------------
간단한 Pick and Place 동작을 테스트하는 스크립트

사용법:
    python3 test_pick_place.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import time
import math

class PickPlaceTest(Node):
    def __init__(self):
        super().__init__('pick_place_test')
        
        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, '/dsr01/target_pose', 10)
        self.target_joint_pub = self.create_publisher(JointState, '/dsr01/target_joint', 10)
        
        # Service Client
        self.gripper_client = self.create_client(SetBool, '/dsr01/set_gripper_state')
        
        self.get_logger().info('Pick and Place Test Node Ready')
        
    def move_joint(self, joint_angles_deg):
        """관절 각도로 이동 (degrees)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [math.radians(d) for d in joint_angles_deg]
        
        self.get_logger().info(f'Moving to joint: {joint_angles_deg}')
        self.target_joint_pub.publish(msg)
        time.sleep(3.0)  # 동작 완료 대기
        
    def move_pose(self, x, y, z):
        """직선 이동 (mm)"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        self.get_logger().info(f'Moving to pose: x={x}, y={y}, z={z}')
        self.target_pose_pub.publish(msg)
        time.sleep(3.0)  # 동작 완료 대기
        
    def gripper_control(self, close=True):
        """그리퍼 제어"""
        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Gripper service not available!')
            return
            
        request = SetBool.Request()
        request.data = close
        
        action = 'CLOSE' if close else 'OPEN'
        self.get_logger().info(f'Gripper {action}')
        
        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info(f'Gripper result: {future.result().message}')
        else:
            self.get_logger().error('Gripper service call failed!')
            
        time.sleep(1.0)  # 그리퍼 동작 완료 대기
        
    def run_pick_and_place(self):
        """Pick and Place 시퀀스 실행"""
        self.get_logger().info('='*50)
        self.get_logger().info('Starting Pick and Place Test')
        self.get_logger().info('='*50)
        
        try:
            # 1. Home 위치로 이동
            self.get_logger().info('[1/9] Moving to Home position...')
            self.move_joint([0, 0, 90, 0, 90, 0])
            
            # 2. 그리퍼 열기
            self.get_logger().info('[2/9] Opening gripper...')
            self.gripper_control(close=False)
            
            # 3. 물체 위치로 접근
            self.get_logger().info('[3/9] Approaching object...')
            self.move_pose(400.0, 100.0, 300.0)
            
            # 4. 하강
            self.get_logger().info('[4/9] Lowering to object...')
            self.move_pose(400.0, 100.0, 150.0)
            
            # 5. 그리퍼 닫기 (물체 잡기)
            self.get_logger().info('[5/9] Grasping object...')
            self.gripper_control(close=True)
            
            # 6. 위로 들어올리기
            self.get_logger().info('[6/9] Lifting object...')
            self.move_pose(400.0, 100.0, 300.0)
            
            # 7. 목표 위치로 이동
            self.get_logger().info('[7/9] Moving to target position...')
            self.move_pose(400.0, -100.0, 300.0)
            
            # 8. 하강
            self.get_logger().info('[8/9] Lowering to place...')
            self.move_pose(400.0, -100.0, 150.0)
            
            # 9. 그리퍼 열기 (물체 놓기)
            self.get_logger().info('[9/9] Releasing object...')
            self.gripper_control(close=False)
            
            # 10. Home으로 복귀
            self.get_logger().info('[FINAL] Returning to Home...')
            self.move_joint([0, 0, 90, 0, 90, 0])
            
            self.get_logger().info('='*50)
            self.get_logger().info('Pick and Place Test Completed Successfully!')
            self.get_logger().info('='*50)
            
        except Exception as e:
            self.get_logger().error(f'Error during test: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceTest()
    
    # 2초 대기 (다른 노드들 초기화 대기)
    print('Waiting 2 seconds for other nodes to initialize...')
    time.sleep(2.0)
    
    # Pick and Place 실행
    node.run_pick_and_place()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
