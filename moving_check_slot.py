import time
import rclpy
import DR_init

# =========================
# 로봇 설정 상수 (통일)
# =========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"     # 워크셀에 등록된 툴 무게
ROBOT_TCP  = "GripperDA_v1"    # 워크셀에 등록된 TCP

# =========================
# 속도/가속도 (통일)
# =========================
VELOCITY_FAST = 70
ACC_FAST      = 70
VELOCITY_SLOW = 30
ACC_SLOW      = 30

# 정렬(상대이동/회전/힘제어)에서 쓸 속도
ALIGN_V = 50
ALIGN_A = 50

# 그리퍼 펄스 시간 통일
GRIPPER_PULSE_SEC = 0.3

# =========================
# 슬롯/선반 파라미터 (mm)
# =========================
SLOT_DX = 60.0
SLOT_DY = 60.0
SLOT_DZ = 60.0

Z_UP_MM   = 100.0
X_PULL_MM = -100.0
INSPECT_TIME = 4.0

# =========================
# 기준 포즈 (posx: mm, deg)
# =========================
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

# =========================
# DR_init 설정 (통일)
# =========================
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =========================
# 초기화
# =========================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY_FAST: {VELOCITY_FAST}, ACC_FAST: {ACC_FAST}")
    print(f"VELOCITY_SLOW: {VELOCITY_SLOW}, ACC_SLOW: {ACC_SLOW}")
    print(f"ALIGN_V: {ALIGN_V}, ALIGN_A: {ALIGN_A}")
    print("#" * 50)


# =========================
# 그리퍼 IO (set_digital_output 통일)
# =========================
def open_gripper():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(2, ON)
    wait(GRIPPER_PULSE_SEC)
    set_digital_output(2, OFF)


def close_gripper():
    from DSR_ROBOT2 import set_digital_output, ON, OFF, wait
    set_digital_output(1, ON)
    wait(GRIPPER_PULSE_SEC)
    set_digital_output(1, OFF)


# =========================
# 슬롯 포즈 계산 (통일)
# =========================
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


# =========================
# 이동 래퍼 (통일)
# =========================
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


# =========================
# Force/Compliance (두번째 코드 기능 통합)
# =========================
def start_force():
    from DSR_ROBOT2 import task_compliance_ctrl, set_stiffnessx, set_desired_force, DR_FC_MOD_ABS, wait
    task_compliance_ctrl()
    set_stiffnessx([1000, 1000, 1000, 100, 100, 100])
    set_desired_force([0, 0, -20, 0, 0, 0],
                      [0, 0,  1, 0, 0, 0],
                      mod=DR_FC_MOD_ABS)
    wait(0.2)


def end_force():
    from DSR_ROBOT2 import release_force, release_compliance_ctrl
    release_force(time=0.0)
    release_compliance_ctrl()


def z_up_rel():
    from DSR_ROBOT2 import movel, posx, DR_MV_MOD_REL
    movel(posx(0, 0, 10, 0, 0, 0), v=ALIGN_V, a=ALIGN_A, mod=DR_MV_MOD_REL)


def z_down_rel():
    from DSR_ROBOT2 import movel, posx, DR_MV_MOD_REL
    movel(posx(0, 0, -12, 0, 0, 0), v=ALIGN_V, a=ALIGN_A, mod=DR_MV_MOD_REL)


def xy_check_slot(max_time_sec=6.0, z_success=65.0, fz_contact=1.0):
    """
    두번째 코드의 xy 탐색 + 안전장치(타임아웃/성공조건) 추가.
    """
    from DSR_ROBOT2 import (
        get_current_posx, get_tool_force,
        movel, posx,
        DR_BASE, DR_MV_MOD_REL,
        wait
    )

    start_t = time.time()
    start_force()

    while rclpy.ok():
        # 타임아웃
        if time.time() - start_t > max_time_sec:
            break

        pose, _ = get_current_posx(DR_BASE)
        x, y, z, rx, ry, rz = pose
        fx, fy, fz, tx, ty, tz = get_tool_force()

        # 성공 조건(예: 충분히 들어갔고 접촉 힘이 안정적)
        if z <= z_success and fz >= fz_contact:
            break

        # 문제 감지 + 보정 이동
        if fz >= 5 and z > z_success:
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

            end_force()
            z_up_rel()
            movel(posx(dx, dy, 0, 0, 0, 0), v=ALIGN_V, a=ALIGN_A, mod=DR_MV_MOD_REL)
            z_down_rel()
            start_force()

        wait(0.05)

    end_force()


def spiral_check_slot():
    from DSR_ROBOT2 import get_tool_force, movej, DR_MV_MOD_REL, wait

    best_angle = 0.0
    min_torque = float("inf")

    start_force()

    # 제자리 Rz 스캔
    for angle in [-3, -2, -1, 0, 1, 2, 3]:
        end_force()

        movej([0, 0, 0, 0, 0, angle], mod=DR_MV_MOD_REL)
        wait(0.1)

        fx, fy, fz, tx, ty, tz = get_tool_force()
        torque_score = abs(tx) + abs(ty)

        if torque_score < min_torque:
            min_torque = torque_score
            best_angle = angle

        # 누적 회전 방지(원위치 복귀)
        movej([0, 0, 0, 0, 0, -angle], mod=DR_MV_MOD_REL)
        wait(0.05)

        start_force()

    end_force()
    # 최적 각도로 정렬
    movej([0, 0, 0, 0, 0, best_angle], mod=DR_MV_MOD_REL)


def check_slot():
    """
    두번째 코드의 check_slot을 그대로 사용하되,
    - 문제 없을 때도 force 모드 해제되도록 수정(안전)
    """
    from DSR_ROBOT2 import get_current_posx, get_tool_force, movej, DR_BASE, DR_MV_MOD_REL, wait

    Z_SUCCESS = 65.0
    FZ_CONTACT = 2.0
    TORQUE_TH = 0.6
    ROT_TEST_ANGLE = 1.0
    ROT_RESPONSE_TH = 0.3

    start_force()

    # 접촉 대기(너 코드 흐름 유지)
    while rclpy.ok():
        _, _, fz1, _, _, _ = get_tool_force()
        if fz1 >= 0.5:
            break

    pose, _ = get_current_posx(DR_BASE)
    x, y, z, rx, ry, rz = pose
    fx, fy, fz, tx, ty, tz = get_tool_force()

    problem_detected = (
        z > Z_SUCCESS and
        fz > FZ_CONTACT and
        (abs(tx) + abs(ty)) > TORQUE_TH
    )

    if not problem_detected:
        end_force()
        return

    end_force()

    # 회전 반응 테스트
    movej([0, 0, 0, 0, 0, ROT_TEST_ANGLE], mod=DR_MV_MOD_REL)
    wait(0.1)
    fx2, fy2, fz2, tx2, ty2, tz2 = get_tool_force()

    # 원래 각도로 복귀
    movej([0, 0, 0, 0, 0, -ROT_TEST_ANGLE], mod=DR_MV_MOD_REL)
    wait(0.1)

    delta_torque = abs(tx2 - tx) + abs(ty2 - ty)

    if delta_torque > ROT_RESPONSE_TH:
        spiral_check_slot()
    else:
        xy_check_slot()


# =========================
# 작업 시퀀스 (첫번째 코드 기능 + check_slot 삽입)
# =========================
def pick_from_shelf1(base_tup, idx, axis, x_pull_mm):
    from DSR_ROBOT2 import wait

    open_gripper()
    wait(0.2)

    go_slot_xpull(base_tup, idx, axis, x_pull_mm)

    close_gripper()
    wait(0.3)

    leave_slot_xpull(base_tup, idx, axis, x_pull_mm)


def inspect_and_align_at_shelf2(base_tup, idx, axis):
    """
    Shelf2에서:
    1) z-up 접근 후 작업 위치로 이동
    2) check_slot()로 정렬/보정
    3) INSPECT_TIME 대기
    4) z-up으로 이탈
    """
    from DSR_ROBOT2 import wait

    go_slot_zup(base_tup, idx, axis)

    # ✅ 병합 포인트: 여기서 정렬 알고리즘 수행
    check_slot()

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
    node = rclpy.create_node("shelf_pick_align_inspect_place", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    initialize_robot()

    # slot index
    s1_idx = 0
    s2_idx = 0
    s3_idx = 0

    go_home()

    # 1) Shelf1 pick
    pick_from_shelf1(S1_BASE, s1_idx, S1_AXIS, X_PULL_MM)
    go_home()

    # 2) Shelf2 align + inspect
    inspect_and_align_at_shelf2(S2_BASE, s2_idx, S2_AXIS)

    # 3) Shelf3 place
    place_to_shelf3(S3_BASE, s3_idx, S3_AXIS)
    go_home()

    do_homing()
    go_home()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
