#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project H.I.T - Robot Interface Node (Refactored)
-------------------------------------------------
Author: Project Team
Role: Hardware Interface (Body)
Description:
    - Object-Oriented Design for better modularity.
    - Manages Doosan Robot connection via DSR_ROBOT2.
    - Publishes: 
        1. /force_torque_data (WrenchStamped)
        2. /robot_pose (PoseStamped)
        3. /joint_states (JointState)
        4. /robot_system_state (Int32)
    - Subscribes: /target_pose (PoseStamped)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import time
import math

# --- DRL Init Setup ---
import DR_init

# Configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class RobotInterface(Node):
    def __init__(self):
        super().__init__('robot_interface_node', namespace=ROBOT_ID)
        self.get_logger().info(f'Initializing Robot Interface Node in namespace: {ROBOT_ID}...')
        
        self.dsr = None
        self.initialized = False
        
        # Publishers (Safe to create early)
        self.pub_ft = self.create_publisher(WrenchStamped, '/force_torque_data', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_state = self.create_publisher(Int32, '/robot_system_state', 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_cb, 10)
        self.create_subscription(JointState, '/target_joint', self.target_joint_cb, 10)
        
        # Service Servers
        from std_srvs.srv import SetBool # Reuse SetBool for simple On/Off
        self.srv_force_mode = self.create_service(SetBool, '/apply_force_control', self.force_control_cb)
        
        # Mocking State
        self.use_fake_force = False
        self.fake_force_value = 0.0
        
    def set_dsr(self, dsr_module):
        """Receive the imported DSR module from main"""
        self.dsr = dsr_module
        self.get_logger().info("DSR Module linked successfully to RobotInterface.")

    def initialize_robot(self):
        """Configure Tool and TCP settings"""
        if not self.dsr:
            return False
            
        try:
            self.dsr.set_tool("Tool Weight")
            self.dsr.set_tcp("GripperDA_v1")
            self.get_logger().info(f"Robot Initialized: ID={ROBOT_ID}, Model={ROBOT_MODEL}")
            self.initialized = True
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot: {e}")
            return False

            
    def target_pose_cb(self, msg: PoseStamped):
        """Execute Linear motion (movel)"""
        if not self.dsr or not self.initialized:
            self.get_logger().warn("Motion command received but robot not ready.")
            return

        try:
            self.get_logger().info("Received Target Pose, Executing movel...")
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Using posx from dsr module
            target_pos = self.dsr.posx([x, y, z, 0, 180, 0])
            self.dsr.movel(target_pos, vel=20, acc=20)
            
        except Exception as e:
            self.get_logger().error(f"Error executing movel: {e}")

    def target_joint_cb(self, msg: JointState):
        """Execute Joint motion (movej)"""
        if not self.dsr or not self.initialized:
            self.get_logger().warn("Joint command received but robot not ready.")
            return

        try:
            self.get_logger().info(f"Received Target Joint, Executing movej... {msg.position}")
            # ROS uses Radians, DSR uses Degrees. Must Convert!
            target_deg = [math.degrees(rad) for rad in msg.position]
            
            # Pad with zeros if less than 6 joints provided (Safety)
            while len(target_deg) < 6:
                target_deg.append(0.0)

            self.dsr.movej(target_deg[:6], vel=30, acc=30)

        except Exception as e:
            self.get_logger().error(f"Error executing movej: {e}")

    def publish_force_data(self):
        """Fetch and publish Force/Torque data"""
        try:
            # [Mocking Logic]
            if self.use_fake_force:
                msg = WrenchStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "tool_link"
                # Mock Z-force (e.g. simulate contact)
                msg.wrench.force.z = self.fake_force_value 
                self.pub_ft.publish(msg)
                return

            if not self.dsr: return

            ft_list = self.dsr.get_tool_force()
            
            if ft_list and len(ft_list) == 6:
                msg = WrenchStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "tool_link"
                msg.wrench.force.x = float(ft_list[0])
                msg.wrench.force.y = float(ft_list[1])
                msg.wrench.force.z = float(ft_list[2])
                msg.wrench.torque.x = float(ft_list[3])
                msg.wrench.torque.y = float(ft_list[4])
                msg.wrench.torque.z = float(ft_list[5])
                
                self.pub_ft.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"FT Poll Error: {e}", throttle_duration_sec=5.0)

    def force_control_cb(self, request, response):
        """
        Service Callback for /apply_force_control (SetBool)
        True: Enable Compliance (-30N Z-force)
        False: Disable Compliance (Position Mode)
        """
        try:
            if request.data: # Enable
                self.get_logger().info("[Service] Force/Compliance Mode: STARTED")
                
                # Mocking Activation
                self.use_fake_force = True 
                self.fake_force_value = 10.0 # Mock Contact Force
                
                if self.dsr and self.initialized:
                    # Apply real compliance (Default params from no1)
                    stiffness = [500, 500, 500, 100, 100, 100]
                    self.dsr.task_compliance_ctrl(stiffness, [10]*6, 0.0, 1) # 1=ON
                    self.dsr.set_desired_force([0,0,-30,0,0,0], [0,0,1,0,0,0], 0.0, 1) # -30N Z-axis
                    
                response.success = True
                response.message = "Force Control ON (Mocking Active)"
                
            else: # Disable
                self.get_logger().info("[Service] Force/Compliance Mode: STOPPED")
                
                # Mocking Deactivation
                self.use_fake_force = False
                self.fake_force_value = 0.0
                
                if self.dsr and self.initialized:
                    self.dsr.release_force(0.1)
                    self.dsr.release_compliance_ctrl()
                    
                response.success = True
                response.message = "Force Control OFF"
                
        except Exception as e:
            response.success = False
            response.message = f"Failed to switch mode: {e}"
            self.get_logger().error(f"Service Error: {e}")
            
        return response

    def publish_pose_data(self):
        """Fetch and publish Current Robot Pose"""
        if not self.dsr: return

        try:
            # get_current_posx usually returns tuple (pos_list, status)
            # But sometimes might behave differently or be malformed
            raw_data = self.dsr.get_current_posx()
            pos_list = None
            
            # Case 1: Standard Tuple (list, status)
            if isinstance(raw_data, tuple) and len(raw_data) >= 1:
                possible_list = raw_data[0]
                if isinstance(possible_list, (list, tuple)) and len(possible_list) >= 6:
                    pos_list = possible_list

            # Case 2: Direct List (Safety fallback)
            elif hasattr(raw_data, '__iter__') and len(raw_data) >= 6:
                 # Ensure it's not a tuple containing something else
                 if not isinstance(raw_data[0], (list, tuple)): 
                     pos_list = raw_data

            if pos_list:
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                try:
                    msg.pose.position.x = float(pos_list[0])
                    msg.pose.position.y = float(pos_list[1])
                    msg.pose.position.z = float(pos_list[2])
                    
                    # Optional: Orientation (for now just Identity or placeholder if needed)
                    # msg.pose.orientation... 
                    
                    self.pub_pose.publish(msg)
                except (IndexError, TypeError, ValueError):
                    pass # Skip malformed frames safely
        except Exception as e:
            self.get_logger().warn(f"Pose Poll Error: {e}", throttle_duration_sec=5.0)

    def publish_joint_data(self):
        """Fetch and publish Joint States (converts degrees to radians)"""
        if not self.dsr: return

        try:
            # get_current_posj returns DR_common2.posj (list-like) directly
            # NOT a tuple (pos_list, status) like posx
            raw_data = self.dsr.get_current_posj()
            joint_list = None
            
            # Check if raw_data itself is list-like
            if raw_data and hasattr(raw_data, '__iter__') and len(raw_data) >= 6:
                joint_list = raw_data
            
            if joint_list:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                
                # Convert Degrees to Radians for ROS standard
                msg.position = [math.radians(float(j)) for j in joint_list[:6]]
                
                self.pub_joint.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Joint Poll Error: {e}", throttle_duration_sec=5.0)

    def publish_system_state(self):
        """Fetch and publish Robot System State"""
        if not self.dsr: return

        try:
            # get_robot_state returns int
            state = self.dsr.get_robot_state()
            
            if state is not None:
                msg = Int32()
                msg.data = int(state)
                self.pub_state.publish(msg)
                
        except Exception as e:
            self.get_logger().warn(f"State Poll Error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    
    # 1. Create Node First
    node = RobotInterface()
    
    # 2. Setup DSR Import (Manual Injection logic)
    # This mirrors the successful script structure: Init Node -> Link to DR_Init -> Import DSR
    try:
        DR_init.__dsr__node = node
        import DSR_ROBOT2
        node.set_dsr(DSR_ROBOT2)
        node.get_logger().info("DSR_ROBOT2 imported and linked via main.")
    except Exception as e:
        node.get_logger().error(f"CRITICAL: Failed to import DSR library: {e}")
        return

    node.get_logger().info("Entering Main Control Loop (polling every 2.0s)...")
    
    # Loop Rate Control
    loop_rate_sec = 2.0 # 0.5Hz
    last_loop_time = time.time()
    
    is_robot_setup = False

    try:
        while rclpy.ok():
            # 1. Process ROS callbacks (Non-blocking)
            rclpy.spin_once(node, timeout_sec=0.01)
            
            current_time = time.time()
            if (current_time - last_loop_time) >= loop_rate_sec:
                last_loop_time = current_time
                
                # 2. Lazy Initialization (Ensures node is valid for DSR calls if they need it)
                if not is_robot_setup:
                    if node.initialize_robot():
                        is_robot_setup = True
                    continue # Skip data publishing on the init frame
                
                # 3. Publish Data
                if is_robot_setup:
                    node.publish_force_data()
                    node.publish_pose_data()
                    node.publish_joint_data()
                    node.publish_system_state()
                    
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Robot Interface...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


