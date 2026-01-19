#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Direct Robot Movement Control via ROS2
Uses dsr_bringup2 ROS2 interface directly (no project_hit intermediary needed)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
import math

# Doosan specific
from dsr_msgs2.msg import JogStamped
from std_srvs.srv import Empty, SetBool

class DirectRobotController(Node):
    def __init__(self):
        super().__init__('direct_robot_controller')
        
        # Check available topics/services from dsr_bringup2
        self.get_logger().info("Initializing Direct Robot Controller")
        
        # Service clients for robot control
        self.srv_set_rob_mode = self.create_client(SetBool, '/dsr01/system/set_robot_mode')
        self.srv_set_velx = self.create_client(SetBool, '/dsr01/motion/set_velx')
        self.srv_power_on = self.create_client(SetBool, '/dsr01/system/power_on')
        
        self.get_logger().info("Services initialized. Waiting for robot to be ready...")
        time.sleep(1)
        
    def power_on_robot(self):
        """Enable robot"""
        try:
            req = SetBool.Request()
            req.data = True
            
            self.get_logger().info("Powering on robot...")
            future = self.srv_power_on.call_async(req)
            
            # Wait for response with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5:
                time.sleep(0.1)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"Power on response: {response.success}")
                return response.success
            else:
                self.get_logger().warn("Power on service timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error powering on: {e}")
            return False
    
    def set_robot_mode(self, mode):
        """Set robot mode (True=autonomous, False=manual)"""
        try:
            req = SetBool.Request()
            req.data = mode
            
            self.get_logger().info(f"Setting robot mode to {'Autonomous' if mode else 'Manual'}...")
            future = self.srv_set_rob_mode.call_async(req)
            
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5:
                time.sleep(0.1)
            
            if future.done():
                response = future.result()
                self.get_logger().info(f"Mode set response: {response.success}")
                return response.success
            else:
                self.get_logger().warn("Mode set service timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error setting mode: {e}")
            return False

def main():
    rclpy.init()
    controller = DirectRobotController()
    
    try:
        # Try to enable robot
        print("\n=== Attempting to Enable Robot ===")
        controller.power_on_robot()
        time.sleep(2)
        
        print("\n=== Setting Robot Mode ===")
        controller.set_robot_mode(True)
        time.sleep(1)
        
        print("\nRobot control test complete!")
        print("Check /dsr01/ services and topics for available commands")
        
        # List all services
        print("\n=== Available Services ===")
        rclpy.spin_once(controller, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
