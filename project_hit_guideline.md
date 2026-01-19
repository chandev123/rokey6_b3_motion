## Project H.I.T - ROS2 Node Architecture Specification

--------------------------------------------------------------------------------
1. robot_interface_node (하드웨어 인터페이스)
--------------------------------------------------------------------------------
[ 역할 ]
 - Local PC와 상위제어기(AP) 간의 TCP/IP 통신 담당 (TCP/IP ↔ ROS2 변환)
 - 로봇의 실시간 상태 및 센서 데이터 파싱
 - 프로젝트 핵심: 6축 힘/토크 센서 데이터를 실시간으로 발행해야 함
[ 기능 ]
  1. **Hardware Communication**: `DSR_ROBOT2` 드라이버와 연동 및 초기화 (Tool/TCP 설정)
  2. **Sensor Data Parsing**:
     - `get_current_posx()` → Base 좌표(PoseStamped) 변환
     - `get_current_posj()` → 관절 각도(JointState) 변환 (deg → rad)
     - `get_robot_state()` → 시스템 상태(Int32) 모니터링
     - `get_tool_force()` → 6축 힘/토크(WrenchStamped) 파싱
  3. **Motion Execution**:
     - `target_pose_cb` → `movel()` 실행 (Linear Motion)
     - `target_joint_cb` → `movej()` 실행 (Joint Motion)
  4. **Safety Check**: 초기화 상태 및 연결 여부 실시간 확인

[ Interface ]
  - Publisher:
     1. **Kinematics (운동학)**
        - `/robot_pose`         (msg: PoseStamped)   : Base 기준 TCP 좌표 (posx) - 위치(m) 및 자세(quat)
        - `/joint_states`       (msg: JointState)    : 각 관절의 각도(posj) 및 속도
     
     2. **Dynamics (동역학)**
        - `/force_torque_data`  (msg: WrenchStamped) : End-Effector 기준 6축 힘/토크 (Fx, Fy, Fz, Tx, Ty, Tz)
        - `/joint_torque`       (msg: JointState)    : [옵션] 각 관절별 토크 센서 값
     
     3. **System Status (상태)**
        - `/robot_system_state` (msg: RobotState)    : Servo On/Off, E-Stop, Robot Mode(Manual/Auto) 모니터링
 - Subscriber:
    1. /target_pose        (msg: PoseStamped)   : 이동할 목표 위치 명령 수신

--------------------------------------------------------------------------------
2. gripper_service_node (그리퍼 제어)
--------------------------------------------------------------------------------
[ 역할 ]
 - 2-Finger 그리퍼의 단순 개폐(Open/Close) 동작 수행
 - I/O 제어 신호를 AP로 전송

[ Interface ]
 - Service Server:
    1. /set_gripper_state  (srv: SetBool)       : True(Close/Grasp), False(Open/Release)

--------------------------------------------------------------------------------
3. task_controller_node (제어기/두뇌)
--------------------------------------------------------------------------------
[ 역할 ]
 - 전체 공정의 FSM(Finite State Machine) 관리 (대기→이동→검사→삽입→복귀)
 - [핵심] 어드미턴스 제어 알고리즘 연산 (힘 센서 피드백 → 위치/속도 보정)
 - 특이점(Singularity) 회피 경로 생성

[ 기능 ]
  1. **FSM Management**: 전체 공정 상태(IDLE ↔ APPROACH ↔ INSERTING) 전이 제어
  2. **Motion Planning**:
     - **Gross Motion**: `APPROACH` 상태에서 안전 위치(`movej`)로의 큰 이동 명령 생성
     - **Fine Motion**: `INSERTING` 상태에서 힘 제어 기반 정밀 이동명령 생성
  3. **Command Processing**: `/ui_command` (START/STOP) 수신 및 비상 정지 처리
  4. **Force Control Logic**: 실시간 힘 데이터(`/force_torque_data`)를 기반으로 한 삽입 판단 알고리즘

[ Interface ]
 - **Motion Control (이동 제어)**
    - **Methods**: `movel` (Linear), `movej` (Joint) 지원
    - **Mode**: 동기(Sync) 및 비동기(Async) 모드 결정 로직 포함
    - **Automation**: Config 기반 자동화 (하단 참조)

 - **Fine Control (정밀 제어)**
    - Subscriber:
       1. `/robot_status`       : 로봇 도착 여부 확인
       2. `/force_torque_data`  : 삽입 시 힘 데이터 모니터링 (임계값 초과 시 제어 개입)
       3. `/ui_command`         : System Bridge로부터 작업 시작/정지 명령 수신
    - Publisher:
       1. `/target_pose`        : 계산된 이동 명령을 robot_interface로 전송
       2. `/task_result`        : 작업 성공/실패 여부 결과 송신
    - Service Client:
       1. `/set_gripper_state`  : 그리퍼 동작 요청

[ Automation Strategy (자동화) ]
 - **Data Management**: 좌표(Waypoint)와 로직(Logic)의 분리
 - **Configuration Plan**: `config/waypoints.yaml` 파일 도입
    - `home`: 초기 대기 자세 (Joint: `[0, 0, 90, 0, 90, 0]`)
    - `approach_offset`: 접근 오프셋 (Linear: `[100, 0, 0]`)
 - **Controller Update**: `pyyaml` 라이브러리를 통한 Config 로드 및 `movej`/`movel` 인자 동적 적용
 - **Workflow**: Controller 실행 시 Config 로드 → FSM 기반 자동 순차 이동


--------------------------------------------------------------------------------
4. system_bridge_node (통합 관제 & DB)
--------------------------------------------------------------------------------
[ 역할 ]
 - 실시간 제어 주기와 무관한 비동기 작업 처리
 - UI(PM)와의 통신 및 DB 로깅 담당 (제어 노드의 부하 분산)

[ Interface ]
 - Subscriber:
    1. /robot_status       : DB 기록용 로봇 상태 수신
    2. /force_torque_data  : DB 기록용 힘 데이터 수신
    3. /task_result        : 작업 결과(성공/실패) 수신 및 DB Insert
 - Publisher:
    1. /ui_command         : UI 버튼 입력을 ROS2 토픽으로 변환하여 제어기에 전달
 - External:
    1. Database Connection : SQL 쿼리 실행 (Insert Log)
    2. UI Socket/API       : UI로 현재 상태 전송

--------------------------------------------------------------------------------
## 핵심 데이터 흐름 (Insertion Task)
[H/W] → (TCP) → [robot_interface] → (/force_torque_data) → [task_controller]
                                                                ↓ (Admittance Calc)
[H/W] ← (TCP) ← [robot_interface] ← (/target_pose) ←---------- [task_controller]

--------------------------------------------------------------------------------
## 5. Robot Control Reference (Doosan Robot Language - DRL)
--------------------------------------------------------------------------------
[ 개요 ]
 - `doosan-robot2` 패키지는 ROS 2 Service를 통해 DRL(두산 로봇 언어) 명령어와 유사한 Python 인터페이스를 제공함
 - 위치: `src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py`
 
[ 주요 명령어 (Python Wrapper) ]
 - **movej(pos, vel, acc)** : 관절(Joint) 보간 이동
    - `pos`: 목표 관절 각도 (float list [j1, j2, j3, j4, j5, j6])
    - `vel`, `acc`: 속도 및 가속도
 - **movel(pos, vel, acc)** : 직선(Linear) 보간 이동
    - `pos`: 목표 좌표 (float list [x, y, z, A, B, C])
 - **movec(pos1, pos2, vel, acc)** : 원호(Circular) 이동
 - **set_velx(v1, v2)** : 작업 속도 설정 (v1: 선속도, v2: 각속도)
 - **set_accx(a1, a2)** : 작업 가속도 설정

[ 사용 예시 ]
```python
from DSR_ROBOT2 import movej, movel, set_velx, set_accx

# 속도/가속도 설정
set_velx(30, 20)  # 30 mm/sec, 20 deg/sec
set_accx(60, 40)

# 이동 명령
movej([0, 0, 90, 0, 90, 0], vel=100, acc=100)
movel([400, 500, 800, 0, 180, 0], vel=[50, 50], acc=[100, 100])
```

[ 참고 자료 (External Manual) ]
 - **Doosan Robotics Programming Manual (V2.12.1)**: [Link](https://v2-manual.scroll.site/ko/v2-programming-manual/2.12.1/publish)
    - **호환성 확인**: 매뉴얼의 API(예: `set_stiffnessx`, `parallel_axis` 등)가 로컬 `DSR_ROBOT2.py`에 동일한 인터페이스로 구현되어 있음을 확인.
    - **사용법**: 매뉴얼의 함수 설명과 인자 순서를 로컬 Python 코드 개발에 그대로 참조 가능.

--------------------------------------------------------------------------------
## 6. Existing Codebase Analysis (Reference: src/project_hit)
--------------------------------------------------------------------------------
[ 개요 ]
 - 기존 팀원들이 작성한 코드 베이스(`src/project_hit`) 전수 조사 및 분석 결과.

[ 1. 핵심 구현 파일 (Core Implementation) ]
 - **`task_controller_node.py`** (구 `ros2_check_slot`)
    - **상태**: ★ **Main Reference** → **Migration Candidate**
    - **내용**: 힘 제어를 이용한 삽입 알고리즘(Search & Insert)의 ROS 2 노드 구현체.
    - **조치**: 본 프로젝트의 `task_controller` 개발 시 베이스 코드로 사용.
 
 - **`ref_tactile_sorting.drl`** (구 `pick_place_sorting.drl`)
    - **상태**: Reference
    - **내용**: 물체 높이 측정 및 크기 분류 로직. 접촉 감지 로직 참조용.

 - **`prototype_motion_v1.drl`** (구 `moving0117.drl`)
    - **상태**: Reference (Motion)
    - **내용**: 최신 수정된 테스트용 모션 스크립트 (Y좌표 수정 반영).

 - **`UI_guide.md`** (Markdown)
    - **상태**: Reference
    - **내용**: 시스템 아키텍처 및 데이터 흐름도.

[ 2. 보관/미사용 파일 (Deprecated / Error Logs) ]
 - 기존 미사용 파일(`check_current`, `moving1.drl`, `movingj.drl` 등)은 모두 제거됨.