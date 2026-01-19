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

# Gripper Configuration (Customize these based on actual wiring)
GRIPPER_IO_INDEX = 1 # Example: Digital Output #1
Signal_GRASP = True  # High to Close
Signal_RELEASE = False # Low to Open

def init_drl(node):
    global dsr
    DR_init.__dsr__node = node
    try:
        import DSR_ROBOT2
        dsr = DSR_ROBOT2
        node.get_logger().info("DSR_ROBOT2 imported successfully for Gripper Node.")
    except Exception as e:
        node.get_logger().error(f"Failed to import DSR_ROBOT2: {e}")

class GripperService(Node):
    def __init__(self):
        super().__init__('gripper_service_node')
        self.srv = self.create_service(SetBool, '/set_gripper_state', self.gripper_cb)
        self.get_logger().info('Gripper Service Node Ready (/set_gripper_state)')

    def gripper_cb(self, request, response):
        global dsr
        
        if dsr is None:
            response.success = False
            response.message = "DSR Library not initialized"
            return response

        try:
            # Determine Action
            target_signal = Signal_GRASP if request.data else Signal_RELEASE
            action_str = "GRASP" if request.data else "RELEASE"
            
            self.get_logger().info(f"Received Request: {action_str}")
            
            # Execute I/O Control
            # set_digital_output(index, callback) - usually returns 0 on success
            # check DSR_ROBOT2 signature: set_digital_output(index, val)
            dsr.set_digital_output(GRIPPER_IO_INDEX, target_signal)
            
            response.success = True
            response.message = f"Gripper {action_str} signal sent to IO #{GRIPPER_IO_INDEX}"
            
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
