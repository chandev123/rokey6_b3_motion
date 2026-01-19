# 🧠 Project H.I.T : System Overview & Specification

**Status:** Active
**Last Updated:** 2026-01-19
**Project Identity:** Haptic Insertion Technology
**Maintainer : MARKCH (cocoffeechan09@gmail.com)** 

---

## 1. 🆔 프로젝트 개요 (Project Identity)
* **명칭:** Project H.I.T (Haptic Insertion Technology)
* **핵심 정의:** 시각(Vision) 정보 없이, 오직 **촉각(Force/Torque Sensor)** 데이터만을 활용하여 보이지 않는 구멍을 탐색하고 정밀하게 조립(Peg-in-Hole)하는 로봇 제어 기술.
* **목표:** 조명 변화나 가려짐(Occlusion)이 심한 실제 공장 환경에서도 작동하는 **'장인의 손끝 감각'** 구현.

---

## 2. 🏛️ 시스템 아키텍처 (System Architecture)
### 설계 원칙
* **Real-time:** 힘 제어를 위한 실시간성 보장.
* **Asynchronous:** 제어 루프를 방해하지 않는 비동기 통신.
* **Modularity:** 기능별 노드 분리.

### ROS2 노드 구성 (Node Map)
1. **`task_controller_node` (Brain 🧠)**
   - 전체 공정의 상태 머신(FSM) 관리.
   - 전략적 판단(회피, 재진입, 성공 판별) 수행.
   
2. **`robot_interface_node` (Body 💪)**
   - 하드웨어(두산 로봇) DRL 제어 및 물리적 이동 수행.
   - 센서 데이터(F/T) 수집 및 가공.

3. **`gripper_service_node` (Hand ✋)**
   - 그리퍼 개폐(Open/Close) 및 I/O 제어 전담.

4. **`system_bridge_node` (Link 🔗)**
   - 사용자 명령 수신(Start/Stop).
   - 작업 결과 및 로그 DB 저장.

---

## 3. 🌊 플로우차트 및 알고리즘 (Logic: Force Checker)
* **알고리즘 명:** Force Checker
* **기반 이론:** VRCC (Virtual Remote Center of Compliance), Torque-based Recovery
* **핵심 로직 흐름 (Control Flow):**

### Step 1: Soft Approach (유연 진입)
- **Action:** 컴플라이언스(Task Compliance) 제어 활성화.
- **Force:** Z축 -30N 하강 (입구 근처 감속 -20N).

### Step 2: Searching & Decision (탐색 및 판단)
- **Success Condition (성공):** - 깊이 `Z <= 65mm` **AND** 반발력 `Fz >= 2N` (바닥 안착).
- **Jamming Detection (재밍 감지):** - 깊이 `Z > 65mm` **AND** 힘 `Fz >= 5N` 발생 시.

### Step 3: Recovery Action (회피 기동)
- **Logic:** 토크($T_x, T_y$) 방향을 역계산하여 구멍 위치 추적.
- **Sequence:** 1. 힘 제어 해제 (Release)
  2. 후퇴 (Z-Up)
  3. 수평 이동 (Shift `dx`, `dy`)
  4. 재진입 (Retry)

### Step 4: Insertion (삽입)
- 성공 판별 후 강한 힘(-40N)으로 밀어 넣기 및 그리퍼 Open.

---

## 4. 📅 프로젝트 일정 (Schedule WBS)
**Note:** 환경 재구축 이슈로 인해 베이스 구축이 최우선 과제임.

### Phase 1: 환경 복구 (Base Setup)
- Ubuntu 22.04 / ROS2 Humble / Docker 설치.
- Git Repository 복구 및 하드웨어 연결 확인.

### Phase 2: 시스템 설계 (Design)
- 시스템 아키텍처 문서화.
- 상세 Flowchart 도식화 (Force Checker Logic).

### Phase 3: 핵심 구현 (Implementation)
- 개발 순서: **Body(Interface)** → **Brain(Controller)** → **Hand/Link**.

### Phase 4: 통합 및 튜닝 (Integration & Tuning)
- 노드 통합 테스트.
- 임계값(Threshold) 최적화 (Force, Depth, Torque).

### Phase 5: 마무리 (Finalization)
- 코드 리팩토링.
- 최종 데모 시연 영상 촬영 및 문서화.