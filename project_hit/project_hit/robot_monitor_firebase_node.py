#!/usr/bin/env python3
"""
ROS2 Robot Monitor Firebase Node
Subscribes to robot topics and stores data in Firebase Realtime Database
Supports emergency controls and data replay
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import firebase_admin
from firebase_admin import credentials, db
import os
import time
import json
from collections import deque
from datetime import datetime
import math

class RobotMonitorFirebase(Node):
    """
    ROS2 Node that bridges robot telemetry to Firebase
    
    Subscriptions:
        - /dsr01/task_status: Robot task status
        - /dsr01/force_torque_data: Force/torque sensor data
        - /dsr01/robot_pose: Robot TCP position
        - /dsr01/system_info: System information
        - /dsr01/operational_time: Running time
        - /dsr01/joint_states: Joint angles (Radian → Degree conversion)
        - /dsr01/robot_logs: Robot log messages
    
    Publishing:
        - /dsr01/emergency_stop: Emergency stop command (if needed)
    """
    
    def __init__(self):
        super().__init__('robot_monitor_firebase_node')
        
        # ========== Configuration ==========
        SERVICE_ACCOUNT_KEY_PATH = os.path.expanduser(
            "~/cobot_ws/src/project_hit/rokey-b-3-firebase-adminsdk-fbsvc-09612845da.json"
        )
        DATABASE_URL = "https://rokey-b-3-default-rtdb.firebaseio.com/"
        
        # ========== Firebase Initialization ==========
        try:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
            self.ref = db.reference('/one')
            self.logs_ref = db.reference('/one/logs')
            self.emergency_ref = db.reference('/one/emergencyStop')
            self.get_logger().info(f"[Firebase Connected] {DATABASE_URL}")
        except ValueError:
            self.ref = db.reference('/one')
            self.logs_ref = db.reference('/one/logs')
            self.emergency_ref = db.reference('/one/emergencyStop')
            self.get_logger().info("[Firebase] Already initialized")
        except Exception as e:
            self.get_logger().error(f"[Firebase Init Failed] {e}")
            return
        
        # ========== Data Subscriptions ==========
        self.create_subscription(String, '/dsr01/task_status', self.cb_status, 10)
        self.create_subscription(Float32MultiArray, '/dsr01/force_torque_data', self.cb_ft, 10)
        self.create_subscription(PoseStamped, '/dsr01/robot_pose', self.cb_pose, 10)
        self.create_subscription(String, '/dsr01/system_info', self.cb_system_info, 10)
        self.create_subscription(Float32MultiArray, '/dsr01/operational_time', self.cb_op_time, 10)
        self.create_subscription(JointState, '/dsr01/joint_states', self.cb_joint_states, 10)
        self.create_subscription(String, '/dsr01/robot_logs', self.cb_robot_logs, 10)
        
        # ========== Emergency Command Listener ==========
        self.emergency_ref.listen(self.cb_emergency_command)
        
        # ========== Data Buffers ==========
        self.current_data = {
            "status": "WAITING",
            "force_z": 0.0,
            "pos": {"x": 0.0, "y": 0.0, "z": 0.0},
            "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # J1-J6 in degrees
            "system_info": {
                "name": "Doosan M0609",
                "version": "1.0.0",
                "device": "Robot-01"
            },
            "operational_time": 0.0,
            "logs": [],  # Robot log messages with timestamps
            "timestamp": time.time()
        }
        
        # ========== Log Storage (600 frames @ 10Hz = 60 seconds) ==========
        self.logs = deque(maxlen=600)
        
        # ========== Robot Message Buffer (최근 10개 메시지 유지) ==========
        self.robot_logs = deque(maxlen=10)
        
        # ========== Upload Timer (0.2s = 5Hz) ==========
        self.create_timer(0.2, self.upload_data)
        
        # ========== Log Recording Timer (0.1s = 10Hz for 600 frames = 60s history) ==========
        self.create_timer(0.1, self.record_log)
        
        self.get_logger().info("🤖 Robot Monitor Firebase Node Started")
        self.get_logger().info(f"   Data Path: /one")
        self.get_logger().info(f"   Upload Rate: 5Hz | Log Rate: 10Hz (600 frames = 60sec)")
    
    # ========== ROS2 Callbacks ==========
    
    def cb_status(self, msg: String):
        """Update robot task status"""
        self.current_data["status"] = msg.data
        self.get_logger().debug(f"[Status] {msg.data}")
    
    def cb_ft(self, msg: Float32MultiArray):
        """Extract Z-axis force from [Fx, Fy, Fz, Mx, My, Mz]"""
        if len(msg.data) >= 3:
            self.current_data["force_z"] = round(msg.data[2], 2)
            self.get_logger().debug(f"[Force] Fz={self.current_data['force_z']}N")
    
    def cb_pose(self, msg: PoseStamped):
        """Update robot TCP position"""
        self.current_data["pos"] = {
            "x": round(msg.pose.position.x, 2),
            "y": round(msg.pose.position.y, 2),
            "z": round(msg.pose.position.z, 2)
        }
        self.get_logger().debug(f"[Pose] ({self.current_data['pos']['x']}, {self.current_data['pos']['y']}, {self.current_data['pos']['z']})")
    
    def cb_system_info(self, msg: String):
        """Update system information"""
        try:
            info = json.loads(msg.data)
            self.current_data["system_info"].update(info)
            self.get_logger().debug(f"[System] {info}")
        except:
            pass
    
    def cb_op_time(self, msg: Float32MultiArray):
        """Update operational time"""
        if len(msg.data) > 0:
            self.current_data["operational_time"] = round(msg.data[0], 1)
            self.get_logger().debug(f"[OpTime] {self.current_data['operational_time']}s")
    
    def cb_joint_states(self, msg: JointState):
        """
        Handle joint states (Radian → Degree conversion)
        Store J1-J6 in degrees
        """
        try:
            if len(msg.position) >= 6:
                # Convert radians to degrees for J1-J6
                self.current_data["joints"] = [
                    round(math.degrees(msg.position[i]), 2) for i in range(6)
                ]
                self.get_logger().debug(f"[Joints] {self.current_data['joints']}")
        except Exception as e:
            self.get_logger().warn(f"Joint states error: {e}")
    
    def cb_robot_logs(self, msg: String):
        """
        Handle robot log messages
        Add timestamp ([HH:MM:SS]) and store in deque (max 10 messages)
        """
        try:
            timestamp = datetime.now().strftime("[%H:%M:%S]")
            log_msg = f"{timestamp} {msg.data}"
            self.robot_logs.append(log_msg)
            self.current_data["logs"] = list(self.robot_logs)
            self.get_logger().info(f"[Log] {log_msg}")
        except Exception as e:
            self.get_logger().warn(f"Robot log error: {e}")
    
    def cb_emergency_command(self, message):
        """Handle emergency stop/recovery commands"""
        try:
            if message is None:
                return
            
            data = message
            if isinstance(data, dict) and "command" in data:
                cmd = data["command"]
                self.get_logger().warn(f"⚠️  EMERGENCY COMMAND: {cmd}")
                
                # TODO: Publish to ROS2 topic for robot control
                # self.emergency_publisher.publish(String(data=cmd))
                
        except Exception as e:
            self.get_logger().error(f"Emergency command error: {e}")
    
    # ========== Firebase Upload ==========
    
    def upload_data(self):
        """Upload current data to Firebase (5Hz)"""
        try:
            self.current_data["timestamp"] = time.time()
            self.ref.update(self.current_data)
            self.get_logger().info(f"✓ Status: {self.current_data['status']} | Fz: {self.current_data['force_z']}N | Pos: ({self.current_data['pos']['x']}, {self.current_data['pos']['y']}, {self.current_data['pos']['z']})")
        except Exception as e:
            self.get_logger().warn(f"Upload Error: {e}")
    
    def record_log(self):
        """Record data to history log (10Hz, 600 frames = 60 seconds)"""
        try:
            log_entry = {
                "timestamp": time.time(),
                "status": self.current_data["status"],
                "pos": self.current_data["pos"].copy(),
                "force_z": self.current_data["force_z"]
            }
            self.logs.append(log_entry)
            
            # Every 60 frames (6 seconds), sync logs to Firebase
            if len(self.logs) % 60 == 0:
                logs_list = list(self.logs)
                self.logs_ref.set(logs_list)
                
        except Exception as e:
            self.get_logger().warn(f"Log record error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitorFirebase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
