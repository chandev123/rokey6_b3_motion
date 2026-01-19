#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project H.I.T - Gripper Service Node
------------------------------------
Author: Project Team
Role: Gripper Control (Hand)
Description:
    - Provides a ROS 2 Service (/set_gripper_state) to control the gripper.
    - Uses DSR_ROBOT2 digital outputs (I/O) to trigger physical gripper.
    - Maps SetBool True -> GRASP (Close), False -> RELEASE (Open).
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

# --- DRL Init Setup ---
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Global Variables
dsr = None
g_ros_node = None

# Gripper Configuration (Dual Digital Output Control)
# DO1, DO2를 이용한 그리퍼 제어
GRIPPER_DO1 = 1  # Digital Output #1
GRIPPER_DO2 = 2  # Digital Output #2
GRIPPER_WAIT_TIME = 0.5  # 그리퍼 동작 대기 시간 (초)

def init_drl(node):
    global dsr
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2
        dsr = DSR_ROBOT2
        node.get_logger().info("DSR_ROBOT2 imported successfully for Gripper Node.")
    except Exception as e:
        node.get_logger().error(f"Failed to import DSR_ROBOT2: {e}")

def grip_close():
    """그리퍼 닫기 (GRASP)"""
    global dsr
    if dsr is None:
        return False
    dsr.set_digital_output(GRIPPER_DO1, 1)   # ON
    dsr.set_digital_output(GRIPPER_DO2, 0)  # OFF
    dsr.wait(GRIPPER_WAIT_TIME)
    return True

def grip_open():
    """그리퍼 열기 (RELEASE)"""
    global dsr
    if dsr is None:
        return False
    dsr.set_digital_output(GRIPPER_DO1, 0)  # OFF
    dsr.set_digital_output(GRIPPER_DO2, 1)   # ON
    dsr.wait(GRIPPER_WAIT_TIME)
    return True

class GripperService(Node):
    def __init__(self):
        super().__init__('gripper_service_node', namespace='dsr01')
        self.srv = self.create_service(SetBool, 'set_gripper_state', self.gripper_cb)
        self.get_logger().info('Gripper Service Node Ready (set_gripper_state)')

    def gripper_cb(self, request, response):
        global dsr
        
        if dsr is None:
            response.success = False
            response.message = "DSR Library not initialized"
            return response

        try:
            action_str = "GRASP (Close)" if request.data else "RELEASE (Open)"
            self.get_logger().info(f"Received Request: {action_str}")
            
            # Execute Gripper Control (Dual Output)
            if request.data:
                result = grip_close()
            else:
                result = grip_open()
            
            if result:
                response.success = True
                response.message = f"Gripper {action_str} completed (DO1, DO2)"
            else:
                response.success = False
                response.message = "Gripper control failed - DSR not ready"
            
        except Exception as e:
            self.get_logger().error(f"Gripper Control Failed: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            
        return response

def main(args=None):
    global g_ros_node
    
    rclpy.init(args=args)
    # 1. dedicated node for DSR (similar to robot_interface)
    dsr_node = rclpy.create_node('dsr_client_gripper', namespace='dsr01')
    
    # 2. Service Node
    service_node = GripperService()
    g_ros_node = service_node
    
    # Setup DRL
    init_drl(dsr_node)
    
    # Spin both nodes (Service needs to respond, DSR needs to send)
    # Since Service Callback blocks until DSR returns, we do need MultiThreaded logic 
    # OR we just rely on the fact that set_digital_output might be async-friendly.
    # However, for simplicity and robustness (as proved in robot_interface), 
    # let's run a simple spin loop or use MultiThreadedExecutor if needed.
    # But wait, SetBool is a service. It MUST return. 
    # If we use `dsr.set_digital_output`, it calls a service on `dsr_node`.
    # So `dsr_node` MUST be spinning to process that request? 
    # NO, dsr_node is the CLIENT. The SERVER is the controller. 
    # But `dsr.set_digital_output` does `call_async`?
    # Let's check DSR implementation again. `future = client.call_async()`. 
    # If it waits for result, we need spin. 
    
    # Safe Strategy: Use MultiThreadedExecutor to spin both
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(dsr_node)
    executor.add_node(service_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        service_node.get_logger().info('Shutting down Gripper Service...')
    finally:
        executor.shutdown()
        service_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
