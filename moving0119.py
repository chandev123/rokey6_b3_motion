import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"     # 워크셀에 등록된 툴 무게
ROBOT_TCP  = "GripperDA_v1"    # 워크셀에 등록된 TCP

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY_FAST = 70
ACC_FAST      = 70
VELOCITY_SLOW = 30
ACC_SLOW      = 30

SLOT_DX = 60.0
SLOT_DY = 60.0
SLOT_DZ = 60.0

Z_UP_MM   = 100.0
X_PULL_MM = -100.0
INSPECT_TIME = 4.0
def _posx_from_tuple(t):
    from DSR_ROBOT2 import posx
    return posx(t[0], t[1], t[2], t[3], t[4], t[5])

HOME_P = _posx_from_tuple((367.22, 3.83, 201.28, 158.41, 179.96, 158.79))

S1_BASE = (750.39,  200.46, 250.92,   2.36,   88.13,   2.04)
S2_BASE = (429.80, -258.59,  86.78, 115.36, -179.93, 116.26)
S3_BASE = (429.06,  213.83,  87.04, 154.82,  179.97, 155.23)

S1_AXIS = "z"
S2_AXIS = "x"
S3_AXIS = "x"


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY_FAST: {VELOCITY_FAST}")
    print(f"ACC_FAST: {ACC_FAST}")
    print(f"VELOCITY_SLOW: {VELOCITY_SLOW}")
    print(f"ACC_SLOW: {ACC_SLOW}")
    print("#" * 50)

def open_gripper():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(2, ON)
    wait(0.3)
    set_digital_output(2, OFF)


def close_gripper():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(1, ON)
    wait(0.3)
    set_digital_output(1, OFF)

def slot_posx(base_tup, idx, axis):
    """slot pose (work)"""
    from DSR_ROBOT2 import posx
    x0, y0, z0, a0, b0, c0 = base_tup
    x, y, z = x0, y0, z0

    if axis == "x":
        x = x0 + idx * SLOT_DX
    elif axis == "y":
        y = y0 + idx * SLOT_DY
    elif axis == "z":
        z = z0 + idx * SLOT_DZ

    return posx(x, y, z, a0, b0, c0)


def app_posx_zup(base_tup, idx, axis):
    """approach pose: z + Z_UP_MM"""
    from DSR_ROBOT2 import posx
    x0, y0, z0, a0, b0, c0 = base_tup
    x, y, z = x0, y0, z0

    if axis == "x":
        x = x0 + idx * SLOT_DX
    elif axis == "y":
        y = y0 + idx * SLOT_DY
    elif axis == "z":
        z = z0 + idx * SLOT_DZ

    return posx(x, y, z + Z_UP_MM, a0, b0, c0)


def app_posx_xpull(base_tup, idx, axis, x_pull_mm):
    """approach pose: x + X_PULL_MM (pull-out)"""
    from DSR_ROBOT2 import posx
    x0, y0, z0, a0, b0, c0 = base_tup
    x, y, z = x0, y0, z0

    if axis == "x":
        x = x0 + idx * SLOT_DX
    elif axis == "y":
        y = y0 + idx * SLOT_DY
    elif axis == "z":
        z = z0 + idx * SLOT_DZ

    return posx(x + x_pull_mm, y, z, a0, b0, c0)


def go_home():
    from DSR_ROBOT2 import movel
    movel(HOME_P, v=VELOCITY_FAST, a=ACC_FAST)


def go_slot_zup(base_tup, idx, axis):
    from DSR_ROBOT2 import movel
    p_app = app_posx_zup(base_tup, idx, axis)
    p_work = slot_posx(base_tup, idx, axis)
    movel(p_app,  v=VELOCITY_FAST, a=ACC_FAST)
    movel(p_work, v=VELOCITY_SLOW, a=ACC_SLOW)


def leave_slot_zup(base_tup, idx, axis):
    from DSR_ROBOT2 import movel
    p_app = app_posx_zup(base_tup, idx, axis)
    movel(p_app, v=VELOCITY_FAST, a=ACC_FAST)


def go_slot_xpull(base_tup, idx, axis, x_pull_mm):
    from DSR_ROBOT2 import movel
    p_app = app_posx_xpull(base_tup, idx, axis, x_pull_mm)
    p_work = slot_posx(base_tup, idx, axis)
    movel(p_app,  v=VELOCITY_FAST, a=ACC_FAST)
    movel(p_work, v=VELOCITY_SLOW, a=ACC_SLOW)


def leave_slot_xpull(base_tup, idx, axis, x_pull_mm):
    from DSR_ROBOT2 import movel
    p_app = app_posx_xpull(base_tup, idx, axis, x_pull_mm)
    movel(p_app, v=VELOCITY_FAST, a=ACC_FAST)


def pick_from_shelf1(base_tup, idx, axis, x_pull_mm):
    from DSR_ROBOT2 import wait

    open_gripper()
    wait(0.2)

    go_slot_xpull(base_tup, idx, axis, x_pull_mm)

    close_gripper()
    wait(0.3)

    leave_slot_xpull(base_tup, idx, axis, x_pull_mm)


def inspect_at_shelf2(base_tup, idx, axis):
    from DSR_ROBOT2 import wait
    go_slot_zup(base_tup, idx, axis)
    wait(INSPECT_TIME)
    leave_slot_zup(base_tup, idx, axis)


def place_to_shelf3(base_tup, idx, axis):
    from DSR_ROBOT2 import wait

    go_slot_zup(base_tup, idx, axis)

    open_gripper()
    wait(0.3)

    leave_slot_zup(base_tup, idx, axis)


def do_homing():
    pass


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("shelf_pick_inspect_place", namespace=ROBOT_ID)

    # DR_init에 노드 설정 (통일)
    DR_init.__dsr__node = node

    # 초기화 1회
    initialize_robot()

    # slot index
    s1_idx = 0
    s2_idx = 0
    s3_idx = 0

    go_home()

    pick_from_shelf1(S1_BASE, s1_idx, S1_AXIS, X_PULL_MM)
    go_home()

    inspect_at_shelf2(S2_BASE, s2_idx, S2_AXIS)

    place_to_shelf3(S3_BASE, s3_idx, S3_AXIS)
    go_home()

    do_homing()
    go_home()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
