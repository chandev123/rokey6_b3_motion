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

# 힘 제어 모드 
def start_force():
    from DSR_ROBOT2 import(task_compliance_ctrl, set_stiffnessx, set_desired_force, DR_FC_MOD_ABS,wait)
    task_compliance_ctrl()
    set_stiffnessx([1000,1000,1000,100,100,100])
    set_desired_force([0,0,-20,0,0,0],
                      [0,0,1,0,0,0],
                      mod=DR_FC_MOD_ABS)
    wait(0.2)

# 힘 제어 해제    
def end_force():
    from DSR_ROBOT2 import(release_force, release_compliance_ctrl)
    release_force(time=0.0)
    release_compliance_ctrl()      

# 2-1번 :xy 탐색
def xy_check_slot():
    from DSR_ROBOT2 import(
    # 힘 제어
    set_desired_force,
    # 위치
    get_current_posx, get_tool_force,
    # 이동
    movel, posx,
    # 상수
    DR_BASE,DR_FC_MOD_ABS,DR_MV_MOD_REL,
    # 정지
    wait
    )
        
    start_force()
    # 구분 알고리즘
    while rclpy.ok():
        # Z위치 변수
        pose, sol = get_current_posx(DR_BASE)
        x, y, z, rx, ry, rz = pose    
        # 외력 측정 변수 (base죄표계 기준)
        fx, fy, fz, tx, ty, tz = get_tool_force()
        
        # # 성공판별 기준
        # if z <= 65 and fz >= 1:
        #     break
        
        # 입구도착 식별을 위한 force
        # if z <= 120:
        #     set_desired_force([0.0, 0.0, -20.0, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)
        #     wait(0.3)  # 힘 제어 안정화
            
        # 1번 조건 : 외력이 있고, z값이 ?이상일떄 분류
        if fz >=5 and z > 65 :
                
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
            end_force()
            # 이동
            z_up()
            movel(posx(dx, dy, 0, 0, 0, 0), v=50,a=50,mod= DR_MV_MOD_REL)
            z_down()
            
            # 힘 제어 모드 활성화
            start_force()
       
# 2-2번: 원 탑색
def spiral_check_slot():
    from DSR_ROBOT2 import(
    # 위치
    get_tool_force,
    # 이동
    movej,
    # 상수
    DR_MV_MOD_REL,
    # 정지
    wait,
    )

    best_angle = 0.0
    # flaot('inf') -> 무한대
    min_torque = float('inf')

    start_force()
    
    # 제자리 Rz 스캔 (±5도)
    for angle in [-3, -2, -1, 0, 1, 2, 3]:
        end_force()

        movej([0,0,0,0,0,angle], mod=DR_MV_MOD_REL)
        wait(0.1)

        fx, fy, fz, tx, ty, tz = get_tool_force()
        torque_score = abs(tx) + abs(ty)

        if torque_score < min_torque:
            min_torque = torque_score
            best_angle = angle

        # 원위치 복귀 (누적 회전 방지)
        movej([0,0,0,0,0,-angle], mod=DR_MV_MOD_REL)
        wait(0.05)
        start_force()

    end_force()
    # 최적 각도로 정렬
    movej([0,0,0,0,0,best_angle], mod=DR_MV_MOD_REL)

# 1번 정렬 탐색
def check_slot():
    from DSR_ROBOT2 import(
    # 위치
    get_current_posx, get_tool_force,
    # 이동
    movej, 
    # 상수
    DR_BASE,DR_MV_MOD_REL,
    # 정지
    wait
    )

    # 수정해야됨
    Z_SUCCESS = 65.0        # 성공 삽입 깊이
    FZ_CONTACT = 2.0       # 접촉 판단 힘
    TORQUE_TH = 0.6        # 문제 감지 토크
    # 정상 삽입 입구의 토크값을 지정(abs(tx)+abs(ty))
    ROT_TEST_ANGLE = 1.0   # 회전 반응 테스트 각도 (deg)
    ROT_RESPONSE_TH = 0.3  # 회전 민감도 기준
    #  정상 삽입 입구의  x,y 토크의 평균값으로 지정

    start_force()

    while rclpy.ok():
        _, _, fz1, _, _, _ = get_tool_force()
        if fz1 >= 0.5:
            break

    # 현재 상태 측정
    pose, _ = get_current_posx(DR_BASE)
    x,y,z,rx,ry,rz = pose
    fx, fy, fz, tx, ty, tz = get_tool_force()

    # 문제 감지 (비스듬함)
    problem_detected = (
        z > Z_SUCCESS and
        fz > FZ_CONTACT and
        (abs(tx) + abs(ty)) > TORQUE_TH
    )

    if not problem_detected:
        return

    end_force()

    # 회전 반응 테스트
    movej([0,0,0,0,0,ROT_TEST_ANGLE], mod=DR_MV_MOD_REL)
    wait(0.1)
    fx2, fy2, fz2, tx2, ty2, tz2 = get_tool_force()

    # 원래 각도로 복귀
    movej([0,0,0,0,0,-ROT_TEST_ANGLE], mod=DR_MV_MOD_REL)
    wait(0.1)
    delta_torque = abs(tx2 - tx) + abs(ty2 - ty)

    # 비스듬함
    if delta_torque > ROT_RESPONSE_TH:
        spiral_check_slot()
    else:
        xy_check_slot()

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
