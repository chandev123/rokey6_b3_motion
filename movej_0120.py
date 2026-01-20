import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"      # 워크셀에 등록된 툴 무게 이름
ROBOT_TCP  = "GripperDA_v1"     # 워크셀에 등록된 TCP 이름

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

INSPECT_TIME = 2.0

V_FAST = 70
A_FAST = 70
V_SLOW = 20
A_SLOW = 20

VJ = 70
AJ = 70

HOME_P_LIST = [367.22, 3.83, 201.28, 158.41, 179.96, 158.79]

q_mid = [26.01, 17.70, 87.27, 0.13, 75.06, 26.03]

S1_PICK_PUSH_X = 20.0
S1_PICK_PULL_X = -10.0
S1_BEFORE_CLOSE_WAIT = 0.5

S3_PITCH_DELTA_DEG = 20.0
S3_PITCH_SIGN = -1.0

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"V_FAST/A_FAST: {V_FAST}/{A_FAST}")
    print(f"V_SLOW/A_SLOW: {V_SLOW}/{A_SLOW}")
    print(f"VJ/AJ: {VJ}/{AJ}")
    print("#" * 50)


def do_homing():
    # from DSR_ROBOT2 import homing
    # homing()
    pass

def open_gripper():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(2, ON)
    wait(1.00)
    set_digital_output(2, OFF)

def close_gripper():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, ON)
    wait(1.00)
    set_digital_output(1, OFF)

q_s1 = [
    [30.62, 18.12, 122.74, -146.41, 57.81, 161.67],
    [40.95, 24.14, 113.02, -133.93, 59.29, 152.80],
    [49.59, 30.45, 102.15, -123.56, 61.19, 144.29],
]
q_s2 = [
    [-31.45, 19.05, 69.88, 0.07, 91.00, -30.92],
    [-28.24, 27.55, 54.35, 0.11, 98.04, -27.66],
    [-25.60, 37.51, 39.08, 0.17, 103.36, -24.97],
]


def goj(q):
    from DSR_ROBOT2 import movej
    movej(q, vel=VJ, acc=AJ)

def go_home():
    from DSR_ROBOT2 import movel, posx
    HOME_P = posx(HOME_P_LIST)
    movel(HOME_P, vel=V_FAST, acc=A_FAST)

def _extract_posx6(p):
    try:
        if len(p) >= 6:
            return p[0], p[1], p[2], p[3], p[4], p[5]
    except Exception:
        pass

    try:
        if len(p) >= 1:
            p0 = p[0]
            if len(p0) >= 6:
                return p0[0], p0[1], p0[2], p0[3], p0[4], p0[5]
    except Exception:
        pass

    
    try:
        from DSR_ROBOT2 import tp_popup
        tp_popup("get_current_posx() format unexpected:\n" + str(p))
    except Exception:
        print("get_current_posx() format unexpected:", p)

    raise RuntimeError("Cannot extract 6D pose from get_current_posx()")

def _wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a

def _pose_from_current():
    from DSR_ROBOT2 import get_current_posx
    p = get_current_posx()
    x, y, z, A, B, C = _extract_posx6(p)
    return (x, y, z, A, B, C)

def _pose_same(p6):
    from DSR_ROBOT2 import posx
    x, y, z, A, B, C = p6
    return posx([x, y, z, A, B, C])

def _pose_tilt_from_pose(p6, pitch_delta):
    from DSR_ROBOT2 import posx
    x, y, z, A, B, C = p6
    B2 = _wrap_deg(B + pitch_delta)
    return posx([x, y, z, A, B2, C])


def move_rel_x(dx, vel=V_SLOW, acc=A_SLOW):
    from DSR_ROBOT2 import get_current_posx, movel, posx
    p = get_current_posx()
    x, y, z, A, B, C = _extract_posx6(p)
    movel(posx([x + dx, y, z, A, B, C]), vel=vel, acc=acc)


def stageA_s1_s2_roundtrip_3cycles_no_home():
    from DSR_ROBOT2 import wait

    go_home()
    wait(0.3)

    for i in range(3):
        open_gripper()
        wait(0.1)

        goj(q_s1[i])
        wait(0.2)

        move_rel_x(S1_PICK_PUSH_X, vel=V_SLOW, acc=A_SLOW)
        wait(S1_BEFORE_CLOSE_WAIT)

        close_gripper()
        wait(0.2)

        move_rel_x(S1_PICK_PULL_X, vel=V_SLOW, acc=A_SLOW)
        wait(0.1)

        goj(q_s2[i])
        wait(0.2)

        wait(INSPECT_TIME)

        open_gripper()
        wait(0.2)


def stageB_s2_s3_pick_qs2_via_qmid_tilt_only_3cycles():
    from DSR_ROBOT2 import wait, movel

    for i in range(3):
        # 1) Shelf2로 이동 후 집기
        goj(q_s2[i])
        wait(0.2)

        close_gripper()
        wait(0.2)

        # Shelf3 중간점 으로 이동
        goj(q_mid)
        wait(0.2)

        # 현재 지점에서 자세만 tilt -> 놓기 -> 원복
        cur6 = _pose_from_current()
        p_work = _pose_same(cur6)
        p_tilt = _pose_tilt_from_pose(cur6, S3_PITCH_SIGN * S3_PITCH_DELTA_DEG)

        movel(p_tilt, vel=V_SLOW, acc=A_SLOW)
        wait(0.1)

        open_gripper()
        wait(0.2)

        movel(p_work, vel=V_SLOW, acc=A_SLOW)
        wait(0.1)

        # 4) 다음 cycle 안정 시작
        goj(q_mid)
        wait(0.1)


def perform_task():
    print("Performing task...")
    stageA_s1_s2_roundtrip_3cycles_no_home()
    stageB_s2_s3_pick_qs2_via_qmid_tilt_only_3cycles()

    do_homing()
    go_home()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_shelf_pipeline", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
