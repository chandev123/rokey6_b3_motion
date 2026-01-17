#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[두뇌] 메인 상태 머신 노드

전체 작업 순서 관리:
    1. ZEROING    - 중력 보상 초기화
    2. APPROACH   - 작업 위치로 접근
    3. INSERTION  - DDR5 메모리 삽입
    4. VERIFICATION - Pull Test 검증
    5. SORTING    - 성공/실패 분류
"""

import rclpy
from rclpy.node import Node
import DR_init
import numpy as np
from enum import Enum
from typing import Optional
from std_msgs.msg import Float64MultiArray, Bool, String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy


# Configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


class Phase(Enum):
    """State Machine Phases"""
    IDLE = 0
    ZEROING = 1
    APPROACH = 2
    INSERTION = 3
    VERIFICATION = 4
    SORTING = 5
    DONE = 6


class MainStateMachineNode(Node):
    """
    메인 상태 머신 노드
    
    Published Topics:
        ~/current_phase (String): 현재 상태
        ~/reward (Float64MultiArray): 보상 값 [reward]
    
    Subscribed Topics:
        /tactile_sensor/force_compensated: 힘 센서 데이터
        /tactile_sensor/contact_detected: 접촉 감지
    
    Parameters:
        force_success_threshold: Pull Test 성공 임계값 (N)
        force_fail_threshold: Pull Test 실패 임계값 (N)
        success_bin_pos: 성공 빈 위치
        fail_bin_pos: 실패 빈 위치
    """
    
    def __init__(self):
        super().__init__('main_state_machine')
        
        # Declare parameters
        self.declare_parameter('force_success_threshold', 3.0)
        self.declare_parameter('force_fail_threshold', 1.0)
        self.declare_parameter('success_bin_pos', [200.0, 200.0, 300.0, 180.0, 0.0, 180.0])
        self.declare_parameter('fail_bin_pos', [200.0, -200.0, 300.0, 180.0, 0.0, 180.0])
        self.declare_parameter('auto_start', True)
        
        self.force_success_threshold = self.get_parameter('force_success_threshold').value
        self.force_fail_threshold = self.get_parameter('force_fail_threshold').value
        self.success_bin_pos = self.get_parameter('success_bin_pos').value
        self.fail_bin_pos = self.get_parameter('fail_bin_pos').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Import DSR_ROBOT2
        from DSR_ROBOT2 import movel, posx, get_current_posx, DR_BASE
        self._movel = movel
        self._posx = posx
        self._get_current_posx = get_current_posx
        self._DR_BASE = DR_BASE
        
        # State
        self.current_phase = Phase.IDLE
        self.current_force = np.zeros(6)
        self.contact_detected = False
        self.last_reward = 0.0
        self.is_zeroed = False
        
        # Statistics
        self.success_count = 0
        self.fail_count = 0
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Publishers
        self.phase_pub = self.create_publisher(String, '~/current_phase', qos)
        self.reward_pub = self.create_publisher(Float64MultiArray, '~/reward', qos)
        
        # Subscribers
        self.force_sub = self.create_subscription(
            Float64MultiArray,
            '/dsr01/tactile_sensor/force_compensated',
            self.force_callback,
            qos
        )
        self.contact_sub = self.create_subscription(
            Bool,
            '/dsr01/tactile_sensor/contact_detected',
            self.contact_callback,
            qos
        )
        
        # Service clients
        self.home_client = self.create_client(Trigger, '/dsr01/motion_control/move_to_home')
        self.approach_client = self.create_client(Trigger, '/dsr01/motion_control/move_to_approach')
        self.insertion_client = self.create_client(Trigger, '/dsr01/motion_control/execute_insertion')
        self.pull_test_client = self.create_client(Trigger, '/dsr01/motion_control/execute_pull_test')
        
        # Services
        self.start_srv = self.create_service(Trigger, '~/start', self.start_callback)
        self.reset_srv = self.create_service(Trigger, '~/reset', self.reset_callback)
        
        # Main loop timer (10Hz)
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
        self.get_logger().info('MainStateMachineNode initialized')
        self.get_logger().info(f'Success threshold: {self.force_success_threshold}N')
        self.get_logger().info(f'Fail threshold: {self.force_fail_threshold}N')
        
        # Auto start if configured
        if self.auto_start:
            self.current_phase = Phase.ZEROING
    
    def force_callback(self, msg: Float64MultiArray):
        """힘 센서 데이터 수신"""
        self.current_force = np.array(msg.data)
    
    def contact_callback(self, msg: Bool):
        """접촉 감지 수신"""
        self.contact_detected = msg.data
        # Zeroing complete when first force data received
        if not self.is_zeroed:
            self.is_zeroed = True
    
    def start_callback(self, request, response):
        """작업 시작"""
        self.current_phase = Phase.ZEROING
        response.success = True
        response.message = 'State machine started'
        return response
    
    def reset_callback(self, request, response):
        """리셋"""
        self.current_phase = Phase.IDLE
        self.last_reward = 0.0
        self.is_zeroed = False
        response.success = True
        response.message = 'State machine reset'
        return response
    
    def state_machine_callback(self):
        """메인 상태 머신 루프"""
        # Publish current phase
        phase_msg = String()
        phase_msg.data = self.current_phase.name
        self.phase_pub.publish(phase_msg)
        
        if self.current_phase == Phase.IDLE:
            pass  # Waiting for start command
        
        elif self.current_phase == Phase.ZEROING:
            self._execute_zeroing()
        
        elif self.current_phase == Phase.APPROACH:
            self._execute_approach()
        
        elif self.current_phase == Phase.INSERTION:
            self._execute_insertion()
        
        elif self.current_phase == Phase.VERIFICATION:
            self._execute_verification()
        
        elif self.current_phase == Phase.SORTING:
            self._execute_sorting()
        
        elif self.current_phase == Phase.DONE:
            self._publish_reward()
            self.get_logger().info(
                f'Episode done! Success: {self.success_count}, Fail: {self.fail_count}'
            )
            self.current_phase = Phase.IDLE
    
    def _transition_to(self, new_phase: Phase):
        """상태 전환"""
        self.get_logger().info(f'{self.current_phase.name} -> {new_phase.name}')
        self.current_phase = new_phase
    
    def _execute_zeroing(self):
        """Phase 1: Zeroing"""
        if self.is_zeroed:
            self.get_logger().info('Zeroing complete')
            self._transition_to(Phase.APPROACH)
        # Wait for tactile_sensor to complete zeroing
    
    def _execute_approach(self):
        """Phase 2: Approach"""
        if not self.approach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Approach service not available')
            return
        
        future = self.approach_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._approach_done(f))
        self._transition_to(Phase.INSERTION)
    
    def _approach_done(self, future):
        result = future.result()
        self.get_logger().info(f'Approach: {result.message}')
    
    def _execute_insertion(self):
        """Phase 3: Insertion"""
        if not self.insertion_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Insertion service not available')
            return
        
        future = self.insertion_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._insertion_done(f))
        self._transition_to(Phase.VERIFICATION)
    
    def _insertion_done(self, future):
        result = future.result()
        self.get_logger().info(f'Insertion: {result.message}')
    
    def _execute_verification(self):
        """Phase 4: Verification (Pull Test)"""
        if not self.pull_test_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Pull test service not available')
            return
        
        future = self.pull_test_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._verification_done(f))
    
    def _verification_done(self, future):
        """Pull Test 결과 처리"""
        result = future.result()
        self.get_logger().info(f'Pull test: {result.message}')
        
        # Parse pull force from result
        try:
            pull_force = float(result.message.split(':')[1].replace('N', '').strip())
        except:
            pull_force = abs(self.current_force[2])
        
        # Calculate reward
        if pull_force >= self.force_success_threshold:
            self.last_reward = 1.0
            self.success_count += 1
            self.get_logger().info(f'SUCCESS! Reward: +1.0')
        elif pull_force < self.force_fail_threshold:
            self.last_reward = -1.0
            self.fail_count += 1
            self.get_logger().info(f'FAIL! Reward: -1.0')
        else:
            self.last_reward = -0.5
            self.get_logger().info(f'UNCERTAIN: Reward: -0.5')
        
        self._transition_to(Phase.SORTING)
    
    def _execute_sorting(self):
        """Phase 5: Sorting"""
        if self.last_reward >= 1.0:
            target_pos = self.success_bin_pos
            self.get_logger().info('Moving to SUCCESS bin')
        else:
            target_pos = self.fail_bin_pos
            self.get_logger().info('Moving to FAIL bin')
        
        try:
            # Move up first
            current_pos = list(self._get_current_posx()[0])
            current_pos[2] += 100
            self._movel(self._posx(*current_pos), vel=100, acc=100, ref=self._DR_BASE)
            
            # Move to bin
            self._movel(self._posx(*target_pos), vel=100, acc=100, ref=self._DR_BASE)
        except Exception as e:
            self.get_logger().error(f'Sorting error: {e}')
        
        self._transition_to(Phase.DONE)
    
    def _publish_reward(self):
        """보상 발행"""
        msg = Float64MultiArray()
        msg.data = [self.last_reward]
        self.reward_pub.publish(msg)
    
    # =========================================================================
    # DRL Interface (for future agent integration)
    # =========================================================================
    
    def get_observation(self) -> np.ndarray:
        """DRL: 현재 관측값"""
        return self.current_force.copy()
    
    def get_reward(self) -> float:
        """DRL: 현재 보상"""
        return self.last_reward
    
    def is_done(self) -> bool:
        """DRL: 에피소드 종료 여부"""
        return self.current_phase == Phase.DONE


def main(args=None):
    # Initialize DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    rclpy.init(args=args)
    
    # Create base node for DR_init
    base_node = rclpy.create_node('main_state_machine_base', namespace=ROBOT_ID)
    DR_init.__dsr__node = base_node
    
    try:
        node = MainStateMachineNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        base_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
