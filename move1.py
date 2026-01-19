import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotState  # 두산 로봇 상태 메시지

class TactileSensorNode(Node):
    def __init__(self):
        super().__init__('tactile_sensor_node')
        
        # [구독자 설정] 로봇의 상태(RobotState)를 받아오는 귀를 엽니다.
        # 토픽 이름 '/dsr01/msg/robot_state'는 에뮬레이터 기본 설정입니다.
        self.subscription = self.create_subscription(
            RobotState,
            '/dsr01/msg/robot_state',
            self.listener_callback,
            10
        )
        
        # 프로젝트 요구사항: 힘 임계치 5N
        self.FORCE_THRESHOLD = 5.0 
        self.get_logger().info('DDR5 Tactile Handler Started: "Feeling without Seeing"')

    def listener_callback(self, msg):
        # 메시지에서 '외부 토크(External Torque)' 값을 가져옵니다.
        # actual_external_torque는 [Fx, Fy, Fz, Tx, Ty, Tz] 순서입니다.
        torque = msg.actual_external_torque
        
        fx, fy, fz = torque[0], torque[1], torque[2]
        
        # 실시간 모니터링 로그 (너무 빠르면 주석 처리 가능)
        # self.get_logger().info(f'Current Force: x={fx:.2f}, y={fy:.2f}, z={fz:.2f}')

        # [FR-01] 지능형 잼 분류 알고리즘 구현
        self.detect_jam(fx, fy, fz)

    def detect_jam(self, fx, fy, fz):
        # 1. 바닥/평면 충돌 (Surface Hit) [cite: 33]
        # Fz 값이 임계치를 넘으면 바닥에 닿았다고 판단
        if abs(fz) > self.FORCE_THRESHOLD:
            self.get_logger().error(f'[JAM DETECTED] Surface Hit! Fz({fz:.2f}N) > 5N -> Retract Needed')
            
        # 2. 측면 충돌 (Side Wall) [cite: 32]
        # Fx 또는 Fy 값이 임계치를 넘으면 벽에 닿았다고 판단
        elif abs(fx) > self.FORCE_THRESHOLD or abs(fy) > self.FORCE_THRESHOLD:
            self.get_logger().warn(f'[JAM DETECTED] Side Wall Collision! Fx/Fy exceed limit -> Slide Move Needed')

def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
