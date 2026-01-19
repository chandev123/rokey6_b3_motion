#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Debug script to check robot status and capabilities
"""

import rclpy
from rclpy.node import Node
import sys
import time

sys.path.insert(0, '/home/roh/cobot_ws/install/dsr_common2/lib/python3.10/site-packages')

import DR_init

# Set configuration BEFORE creating the node
DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "m0609"

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_robot_node', namespace="dsr01")
        
        # Set node in DR_init FIRST
        DR_init.__dsr__node = self
        
        # NOW import DSR_ROBOT2 after DR_init is properly configured
        try:
            import DSR_ROBOT2
            self.dsr = DSR_ROBOT2
            print("[DEBUG] DSR_ROBOT2 loaded successfully")
            self.get_logger().info("DSR module initialized")
            time.sleep(1)
            self.run_debug()
        except Exception as e:
            self.get_logger().error(f"Failed to import DSR_ROBOT2: {e}")
            import traceback
            traceback.print_exc()
    
    def run_debug(self):
        print("\n=== Robot Status ===")
        try:
            state = self.dsr.get_robot_state()
            print(f"Robot State: {state}")
        except Exception as e:
            print(f"Error: {e}")
        
        print("\n=== Current Position (Cartesian) ===")
        try:
            pos = self.dsr.get_current_posx()
            print(f"get_current_posx() returned type: {type(pos)}")
            print(f"get_current_posx() value: {pos}")
            if isinstance(pos, tuple):
                print(f"  - Element 0 type: {type(pos[0])}, value: {pos[0]}")
                if len(pos) > 1:
                    print(f"  - Element 1 type: {type(pos[1])}, value: {pos[1]}")
        except Exception as e:
            print(f"Error getting current position: {e}")
        
        print("\n=== Current Position (Joint) ===")
        try:
            joints = self.dsr.get_current_posj()
            print(f"get_current_posj() returned type: {type(joints)}")
            print(f"get_current_posj() value: {joints}")
        except Exception as e:
            print(f"Error getting current joints: {e}")
        
        print("\n=== Tool Force/Torque ===")
        try:
            ft = self.dsr.get_tool_force()
            print(f"get_tool_force() returned type: {type(ft)}")
            print(f"get_tool_force() value: {ft}")
        except Exception as e:
            print(f"Error getting tool force: {e}")
        
        print("\n=== Robot Activation Status ===")
        try:
            is_busy = self.dsr.is_robot_busy()
            print(f"Robot Busy: {is_busy}")
        except Exception as e:
            print(f"Error checking robot busy: {e}")
        
        print("\nDone!")

def main():
    rclpy.init()
    node = DebugNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
