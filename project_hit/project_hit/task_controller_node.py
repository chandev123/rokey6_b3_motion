#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project H.I.T - Task Controller Node
------------------------------------
Author: Project Team
Role: Main Controller (Brain)
Description:
    - 6축 힘/토크 센서 피드백 기반의 삽입 작업(Insertion Task) 제어
    - 전체 공정의 FSM(Finite State Machine) 관리
    - 로봇 인터페이스 노드로 목표 위치 명령(Target Pose) 전송

Flow Overview:
    1. [Initialize] 노드 실행 및 초기화 (Sub/Pub/Service 연결)
    2. [Idle] 대기 상태. UI 또는 상위 명령 대기
    3. [Approach] 작업 위치로 접근 (Motion Plan)
    4. [Contact Detection] 힘 센서를 모니터링하며 접촉 감지
    5. [Compliance Control] 접촉 시 어드미턴스(Admittance) 제어로 힘을 일정하게 유지하며 삽입
    6. [Completion Check] 삽입 깊이 및 힘 안정을 확인하여 작업 완료 판단
    7. [Return] 복귀 및 다음 작업 준비

DRL Reference (Doosan Robot Language):
    - 본 노드는 DRL 문법을 직접 사용하지 않고, ROS 2 서비스 래퍼(Python)를 통해 제어합니다.
    - 참고 파일:
        1. src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/single_robot_simple.py
           -> DSR_ROBOT2 모듈(movej, movel 등) 사용 패턴 참조
        2. src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py
           -> 실제 사용 가능한 모든 Python API 정의 (API Reference)
    - 주의사항: DRL의 'task_compliance_ctrl' 등의 힘 제어 명령어는 별도 서비스 호출로 대체될 수 있음.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState # Added for movej
from std_msgs.msg import String
from std_srvs.srv import SetBool
# import custom messages if needed (e.g. RobotStatus)

class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller_node')
        self.get_logger().info('Initializing Task Controller Node...')

        # --- Parameters ---
        # 임계값 및 제어 파라미터 (추후 yaml 로드 방식으로 변경 가능)
        self.contact_force_threshold = 5.0  # N (접촉 감지 기준 힘)
        self.insertion_force_target = 10.0  # N (삽입 시 유지하려는 목표 힘)
        self.safe_force_limit = 30.0        # N (안전 한계치, 초과 시 비상 정지)

        # --- State Variables ---
        self.current_state = "IDLE"         # State: IDLE, APPROACH, SEARCHING, INSERTING, COMPLETED, ERROR
        self.last_force_data = None         # 최신 힘/토크 데이터 저장용
        self.robot_pose = None              # 최신 로봇 위치 저장용
        self.approach_cmd_sent = False      # 접근 명령 전송 여부 확인용

        # --- Publishers ---
        # 1. 로봇에게 목표 위치/자세 명령 전송
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.target_joint_pub = self.create_publisher(JointState, '/target_joint', 10) # Added for movej
        # 2. 작업 결과 모니터링 및 DB 저장용
        self.task_result_pub = self.create_publisher(String, '/task_result', 10)

        # --- Subscribers ---
        # 1. 로봇의 현재 상태 (관절각, End-Effector 좌표 등) 수신
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_cb, 10)
        
        # 2. [핵심] 6축 힘/토크 센서 데이터 수신
        self.ft_data_sub = self.create_subscription(WrenchStamped, '/force_torque_data', self.force_callback, 10)

        # 3. UI 또는 시스템 브릿지로부터의 시작/정지 명령
        self.ui_cmd_sub = self.create_subscription(String, '/ui_command', self.ui_command_cb, 10)

        # --- Service Clients ---
        # 1. 그리퍼 제어 요청
        self.gripper_client = self.create_client(SetBool, '/set_gripper_state')
        # while not self.gripper_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn('Gripper service not available, waiting...')

        # --- Main Control Loop ---
        # 10Hz ~ 100Hz 주기로 제어 알고리즘(FSM) 실행
        self.timer = self.create_timer(0.01, self.control_loop) # 100Hz

        self.get_logger().info('Task Controller Node Initialized. Ready.')


    def force_callback(self, msg: WrenchStamped):
        """
        힘 센서 데이터 콜백 함수
        - 실시간으로 들어오는 Fx, Fy, Fz 등을 모니터링
        - 안전 한계치(Safe Limit) 초과 시 즉시 정지 로직 포함 가능
        """
        self.last_force_data = msg
        
        # [Safety Check] (예시)
        # force_magnitude = (msg.wrench.force.x**2 + ... )**0.5
        # if force_magnitude > self.safe_force_limit:
        #     self.emergency_stop()

    def robot_pose_cb(self, msg: PoseStamped):
        """로봇 현재 위치 업데이트"""
        self.robot_pose = msg

    def ui_command_cb(self, msg: String):
        """
        UI 명령 처리
        - "START": 작업 시작
        - "STOP": 작업 중단/초기화
        """
        cmd = msg.data.upper()
        if cmd == "START":
            if self.current_state == "IDLE":
                self.current_state = "APPROACH"
                self.get_logger().info("Task Started: Moving to APPROACH state.")
        elif cmd == "STOP":
            self.current_state = "IDLE"
            self.get_logger().info("Task Stopped: Reset to IDLE.")


    def control_loop(self):
        """
        [Main FSM Loop]
        상태에 따라 판단하고 적절한 제어 명령을 생성
        """
        if self.current_state == "IDLE":
            self.do_idle()

        elif self.current_state == "APPROACH":
            self.do_approach()

        elif self.current_state == "SEARCHING":
            # 구멍 탐색 (Spiral Search 등)
            self.do_search()

        elif self.current_state == "INSERTING":
            # 힘 제어 기반 삽입 (Admittance Control)
            self.do_insertion_with_force_control()

        elif self.current_state == "COMPLETED":
            self.get_logger().info("Task Completed Successfully.")
            self.current_state = "IDLE"

        elif self.current_state == "ERROR":
            self.handle_error()


    # --- Action Methods (To be implemented) ---
    def do_idle(self):
        pass

    def do_approach(self):
        # 1. 위치 수신 대기
        if self.robot_pose is None:
            if not getattr(self, '_waiting_log_printed', False):
                self.get_logger().info("Waiting for Robot Pose data...")
                self._waiting_log_printed = True
            return

        # 2. 명령 생성 (1회만 실행) -> Safe Motion using movej
        if not self.approach_cmd_sent:
            # Safe Home Position (Radians) for movej
            # Example: Joint 1~6 [0, 0, 90, 0, 90, 0] degrees -> Radians
            import math
            safe_home_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0] 
            safe_home_rad = [math.radians(d) for d in safe_home_deg]
            
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.position = safe_home_rad
            
            self.target_joint_pub.publish(joint_msg)
            
            self.get_logger().info(f"Approach Command Sent (movej): {safe_home_deg} deg")
            self.approach_cmd_sent = True
            self.current_state = "IDLE" # 테스트용: 1회 이동 후 정지
            self.get_logger().info("Motion sent. Returning to IDLE.")

    def do_search(self):
        # TODO: 탐색 알고리즘 (나선형 회전 등)
        # 접촉(Contact) 감지 시 -> self.current_state = "INSERTING"
        pass

    def do_insertion_with_force_control(self):
        """
        [핵심 알고리즘 구역]
        - 현재 힘(last_force_data)과 목표 힘(insertion_force_target)의 오차 계산
        - 어드미턴스 제어 수식 적용: F - F_ref = M*a + B*v + K*x
        - 위치 보정값(delta_pos) 계산하여 Target Pose 업데이트
        """
        if self.last_force_data is None:
            return

        # (Algorithm Placeholder)
        # Fz가 목표치에 도달하도록 Z축 하강 속도 조절
        # X, Y축 힘이 0이 되도록 위치 미세 조정 (Align)
        
        # 검사: 삽입 깊이가 목표에 도달했는가?
        # if depth > target_depth:
        #     self.current_state = "COMPLETED"
        pass

    def handle_error(self):
        self.get_logger().error("Error State. Waiting for manual reset.")


    def emergency_stop(self):
        self.get_logger().error("EMERGENCY STOP TRIGGERED!")
        self.current_state = "ERROR"
        # Stop command to robot_interface

def main(args=None):
    rclpy.init(args=args)
    node = TaskController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Task Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
