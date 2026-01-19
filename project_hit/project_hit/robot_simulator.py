#!/usr/bin/env python3
"""
ROS2 Robot Topics Simulator & Test Node
Simulates robot sensor data to test the Firebase monitor node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import time
import math
import sys

class RobotSimulator(Node):
    """Simulates robot topics for testing"""
    
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Publishers
        self.pub_status = self.create_publisher(String, '/dsr01/task_status', 10)
        self.pub_ft = self.create_publisher(Float32MultiArray, '/dsr01/force_torque_data', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/dsr01/robot_pose', 10)
        self.pub_sysinfo = self.create_publisher(String, '/dsr01/system_info', 10)
        self.pub_optime = self.create_publisher(Float32MultiArray, '/dsr01/operational_time', 10)
        
        # Simulation parameters
        self.start_time = time.time()
        self.cycle = 0
        self.position_cycle = 0
        
        # Timer for simulation (10Hz)
        self.create_timer(0.1, self.simulate)
        
        self.get_logger().info("🤖 Robot Simulator Started (10Hz)")
        self.get_logger().info("   Publishing to /dsr01/* topics")
    
    def simulate(self):
        """Simulate robot data"""
        self.cycle += 1
        self.position_cycle = (self.position_cycle + 1) % 100
        
        # 1. Task Status (cycles through different states)
        status_states = ["WAITING", "INSERTING", "SEARCHING", "COMPLETED"]
        status_idx = (self.cycle // 50) % len(status_states)
        status_msg = String()
        status_msg.data = status_states[status_idx]
        self.pub_status.publish(status_msg)
        
        # 2. Force/Torque Data [Fx, Fy, Fz, Mx, My, Mz]
        # Simulate oscillating force
        force_z = 20.0 + 15.0 * math.sin(self.cycle * 0.05)  # Oscillates 5-35 N
        
        ft_msg = Float32MultiArray()
        ft_msg.data = [
            5.0 + 3.0 * math.sin(self.cycle * 0.02),  # Fx
            2.0 + 2.0 * math.cos(self.cycle * 0.03),  # Fy
            force_z,                                    # Fz (main)
            1.0 * math.sin(self.cycle * 0.01),        # Mx
            0.5 * math.cos(self.cycle * 0.02),        # My
            0.2 * math.sin(self.cycle * 0.03)         # Mz
        ]
        self.pub_ft.publish(ft_msg)
        
        # 3. Robot Pose (circular motion)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        
        radius = 50.0  # 50mm radius
        angle = self.position_cycle * 2 * math.pi / 100  # Full rotation in 100 cycles
        
        pose_msg.pose.position.x = 300.0 + radius * math.cos(angle)
        pose_msg.pose.position.y = 400.0 + radius * math.sin(angle)
        pose_msg.pose.position.z = 250.0 + 20.0 * math.sin(self.cycle * 0.02)
        
        pose_msg.pose.orientation.w = 1.0  # Identity quaternion
        self.pub_pose.publish(pose_msg)
        
        # 4. System Info (publish once per second)
        if self.cycle % 10 == 0:
            sysinfo_msg = String()
            sysinfo_msg.data = '{"name": "Doosan M0609", "version": "1.0.0", "device": "Robot-01"}'
            self.pub_sysinfo.publish(sysinfo_msg)
        
        # 5. Operational Time (elapsed time in seconds)
        optime_msg = Float32MultiArray()
        elapsed = time.time() - self.start_time
        optime_msg.data = [elapsed]
        self.pub_optime.publish(optime_msg)
        
        # Log every 10 cycles (1 second)
        if self.cycle % 10 == 0:
            self.get_logger().info(
                f"Cycle {self.cycle}: Status={status_msg.data} | "
                f"Fz={force_z:.1f}N | "
                f"Pos=({pose_msg.pose.position.x:.0f}, {pose_msg.pose.position.y:.0f}, {pose_msg.pose.position.z:.0f})"
            )


def main(args=None):
    rclpy.init(args=args)
    simulator = RobotSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("🛑 Simulator stopped")
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
