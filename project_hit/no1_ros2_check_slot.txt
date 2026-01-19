import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight" #워크셀에 등록된 툴 무게
ROBOT_TCP = "GripperDA_v1" #워크셀에 등록된 그리퍼 이름

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 설정된 설정값 출력
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)

# 그리퍼 열기 함수
def open_gripper():
    from DSR_ROBOT2 import(set_digital_output,ON,OFF,wait)
    set_digital_output(2, ON)
    wait(1.00)
    set_digital_output(2, OFF)

# 그리퍼 닫기 함수
def close_gripper():
    from DSR_ROBOT2 import(set_digital_output,ON,OFF,wait)    
    set_digital_output(1, ON)
    wait(1.00)
    set_digital_output(1, OFF)

# Z축 상승 함수
def z_up():
    from DSR_ROBOT2 import(movel,posx,DR_MV_MOD_REL)
    movel(posx(0,0,10,0,0,0),v=50,a=50,mod= DR_MV_MOD_REL)
#Z축 하강
def z_down():
    from DSR_ROBOT2 import(movel,posx,DR_MV_MOD_REL)
    movel(posx(0,0,-12,0,0,0),v=50,a=50,mod= DR_MV_MOD_REL)
#movel(posx(426.10,13.48,144.04,173.681,-179.048,174.517),v = 50,a=50,mod=DR_MV_MOD_ABS)


# 로봇이 수행할 작업
def check_slot():
    # 필요한 기능 import
    from DSR_ROBOT2 import(
        # 힘 제어
        task_compliance_ctrl,set_stiffnessx,
        set_desired_force,release_force,
        release_compliance_ctrl,
        # 위치
        get_current_posx, get_tool_force,
        # 이동
        movel, posx,
        # 상수
        DR_BASE,DR_FC_MOD_ABS,DR_MV_MOD_REL,
        # 정지
        wait,
     )

    # 힘 제어 모드 활성화
    task_compliance_ctrl()
    # Z축 강성 낮게 (외력이 가해지면 부드럽게 멈춤)
    set_stiffnessx([500.0, 500.0, 500.0, 100.0, 100.0, 100.0], time=0.0)
    # Z축으로 계속 힘을 줘서 자동으로 하강
    set_desired_force([0.0, 0.0, -30.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
    wait(0.3)  # 힘 제어 안정화


    # 구분 알고리즘
    while rclpy.ok():
        # Z위치 변수
        pose, sol = get_current_posx(DR_BASE)
        x, y, z, rx, ry, rz = pose    
        # 외력 측정 변수 (base죄표계 기준)
        fx, fy, fz, tx, ty, tz = get_tool_force()
        
        # 성공판별 기준
        # 변수변경 필요
        if z <= 65 and fz >= 2:
            break
        
        # 입구도착 식별을 위한 force
        # z값 측정 후 변경
        if z <= 120:
            set_desired_force([0.0, 0.0, -20.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
            wait(0.3)  # 힘 제어 안정화
            
        # 1번 조건 : 외력이 있고, z값이 ?이상일떄 분류
        if fz >=5 and z > 65 and tx != ty:
                
            # 3번 조건:  토크값에 따른 상하좌우 이동
            dx = 0
            dy = 0
            if tx > 0:
                dx = +3
            elif tx < 0:
                dx = -3

            if ty > 0:
                dy = +3
            elif ty < 0:
                dy = -3
            # 힘 제어 해제
            release_force(time=0.0)
            release_compliance_ctrl()    
            # 이동
            z_up()
            movel(posx(dx, dy, 0, 0, 0, 0), v=50,a=50,mod= DR_MV_MOD_REL)
            z_down()
            
            # 힘 제어 모드 활성화
            task_compliance_ctrl()
            # Z축 강성 낮게 (외력이 가해지면 부드럽게 멈춤)   
            set_stiffnessx([1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0], time=0.0)
            # Z축으로 계속 힘을 줘서 자동으로 하강
            set_desired_force([0.0, 0.0, -40.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
            wait(0.3)  # 힘 제어 안정화  
        
            
    # 그리퍼 열기
    open_gripper()
    # 상승
    z_up()

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("check_slot", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # try:
    # 초기화는 한 번만 수행
    initialize_robot()

    # 작업 수행 (한 번만 호출)
    check_slot()

    # except KeyboardInterrupt:
    #     print("\nNode interrupted by user. Shutting down...")
    # except Exception as e:
    #     print(f"An unexpected error occurred: {e}")
    # finally:
    rclpy.shutdown()


if __name__ == "__main__":
    main()
