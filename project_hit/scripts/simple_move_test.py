#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple test to move robot arm using target_joint topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class RobotTester(Node):
    def __init__(self):
        super().__init__('robot_tester')
        
        # Publisher to send joint targets
        self.target_joint_pub = self.create_publisher(JointState, '/dsr01/target_joint', 10)
        
        self.get_logger().info('Robot Tester Node Ready')
        
    def move_joints(self, j1, j2, j3, j4, j5, j6):
        """Send joint target in radians"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [
            math.radians(j1),
            math.radians(j2),
            math.radians(j3),
            math.radians(j4),
            math.radians(j5),
            math.radians(j6)
        ]
        
        self.get_logger().info(f'Moving joints to: {[j1, j2, j3, j4, j5, j6]} degrees')
        self.target_joint_pub.publish(msg)

def main():
    rclpy.init()
    tester = RobotTester()
    
    # Wait a bit for subscribers
    time.sleep(2)
    
    try:
        # Home position (common safe position)
        print("\n[1] Moving to Home Position...")
        tester.move_joints(0, 0, 0, 0, 0, 0)
        time.sleep(4)
        
        # Test position
        print("\n[2] Moving to Test Position...")
        tester.move_joints(0, -45, 45, 0, 0, 0)
        time.sleep(4)
        
        # Return to home
        print("\n[3] Returning to Home...")
        tester.move_joints(0, 0, 0, 0, 0, 0)
        time.sleep(2)
        
        print("\nTest completed!")
        
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
