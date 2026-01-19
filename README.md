# 🤖 Project H.I.T - Robot Monitor with Firebase

**실시간 로봇 모니터링 시스템** | 완전한 ROS2 + Firebase + 웹대시보드 통합

> **H.I.T** = Handle-Insert-Task를 위한 Haptic 피드백 기반 협작 로봇 작업 시스템

---

## 📋 System Overview

```
┌─────────────────┐
│  Robot Sensor   │ (Real robot or simulator)
│  (10Hz data)    │
└────────┬────────┘
         │ ROS2 Topics
         ▼
┌─────────────────────────────────────┐
│  robot_monitor_firebase_node.py     │ (Main ROS2 bridge)
│  ✓ Subscribe to 5 robot topics      │
│  ✓ Buffer 60-sec history (600 frames) │
│  ✓ Upload to Firebase at 5Hz        │
│  ✓ Log management                   │
└────────┬────────────────────────────┘
         │ Firebase REST API
         ▼
┌─────────────────────────────────────┐
│  Firebase Realtime Database         │
│  Project: rokey-b-3                 │
│  URL: https://rokey-b-3-...         │
│  Paths: /one, /one/logs             │
└────────┬────────────────────────────┘
         │ REST API polling (500ms)
         ▼
┌─────────────────────────────────────┐
│  Web Dashboard (http://localhost:8001)  │
│  ✓ Real-time status display         │
│  ✓ Force trend graph                │
│  ✓ Position monitoring              │
│  ✓ Emergency controls               │
└─────────────────────────────────────┘
```

---

## 🚀 Quick Start (5분 안에 시작하기)

### 1️⃣ 필수 구성 확인
```bash
cd ~/cobot_ws
source install/setup.bash
```

### 2️⃣ 로봇 시뮬레이터 실행 (테스트 데이터 생성, 10Hz)
```bash
# Terminal 1
ros2 run project_hit robot_simulator
```

### 3️⃣ Firebase 모니터 노드 실행 (데이터 업로드, 5Hz)
```bash
# Terminal 2
ros2 run project_hit robot_monitor_firebase_node
```

### 4️⃣ 웹서버 시작
```bash
# Terminal 3
cd ~/cobot_ws/src/project_hit/frontend
python3 -m http.server 8001
```

### 5️⃣ 웹대시보드 열기
```
브라우저: http://localhost:8001/index.html
```

**예상 결과:**
- 🟢 Connection status: **Connected**
- 📊 Real-time data updates (every 500ms)
- 📈 Force trend graph
- 🟠 Task status changing (WAITING → INSERTING → SEARCHING → COMPLETED)

---

## 📁 Project Structure

```
project_hit/
│
├── 📂 project_hit/           # ROS2 Python Package
│   ├── __init__.py
│   ├── robot_simulator.py           (145 lines)
│   │   └─ 역할: 10Hz 로봇 센서 데이터 생성
│   │   └─ 발행: 5개 ROS2 토픽
│   │   └─ 상태: WAITING → INSERTING → SEARCHING → COMPLETED (반복)
│   │
│   └── robot_monitor_firebase_node.py  (192 lines)
│       └─ 역할: ROS2 → Firebase 메인 브릿지
│       └─ 구독: 5개 로봇 토픽
│       └─ 기능:
│           ✓ 60초 히스토리 버퍼링 (600프레임 @10Hz)
│           ✓ 5Hz로 Firebase 업로드
│           ✓ 긴급 제어 신호 수신
│           ✓ 로그 관리 (6초마다 동기화)
│
├── 📂 frontend/              # 웹대시보드
│   ├── index.html            (221 lines) - Main UI
│   ├── 📂 css/
│   │   └── dashboard.css     (600+ lines) - Dark theme styling
│   ├── 📂 js/
│   │   ├── firebase-rest.js  (250+ lines) - 실시간 폴링 & 데이터 업데이트
│   │   └── firebase-config.js - Firebase 자격증명
│   └── favicon.ico
│
├── 📂 config/                # ROS2 설정
├── 📂 launch/                # ROS2 launch 파일
├── 📂 resource/
│
├── 🔐 rokey-b-3-firebase-adminsdk-fbsvc-*.json  - Firebase 서비스계정
├── package.xml               - ROS2 패키지 매니페스트
├── setup.py                  - Python 패키지 설정
├── setup.cfg
├── README.md                 - 이 파일
└── SYSTEM_ARCHITECTURE.md    - 상세 기술 문서
```

---

## 🔧 주요 구성 요소

### Backend Node: `robot_simulator.py`
**역할**: 실제 로봇 대신 테스트 데이터 생성

**발행하는 토픽:**
- `/dsr01/task_status` (String) - 작업 상태
- `/dsr01/force_torque_data` (Wrench) - 힘/토크
- `/dsr01/robot_pose` (PoseStamped) - 로봇 위치
- `/dsr01/system_info` (String) - 시스템 정보
- `/dsr01/operational_time` (Float64) - 운영 시간

**데이터 특성:**
- 주기: 10Hz (0.1초마다)
- 상태 사이클: WAITING(대기) → INSERTING(삽입) → SEARCHING(탐색) → COMPLETED(완료)
- 센서 값: 현실적인 범위 내 무작위 생성

---

### Backend Node: `robot_monitor_firebase_node.py`
**역할**: ROS2 ↔ Firebase 양방향 통신

**기능:**
1. **구독** - 5개 로봇 토픽에서 데이터 받음 (10Hz)
2. **버퍼링** - 60초 히스토리 유지 (최대 600프레임)
3. **업로드** - Firebase에 현재 상태 업로드 (5Hz)
4. **로깅** - 로그 데이터를 `/one/logs` 에 동기화
5. **제어** - `/one/emergencyStop` 에서 긴급정지 신호 수신

**Firebase 경로:**
```
rokey-b-3-default-rtdb.firebaseio.com/
├── /one                          # 현재 상태
│   ├── status: "INSERTING"
│   ├── force_z: 12.5
│   ├── pos: {x, y, z}
│   ├── operational_time: 1546.8
│   ├── system_info: {...}
│   ├── timestamp: 1768812737.068
│   └── emergencyStop: false
│
└── /one/logs                      # 60초 히스토리 (600개 항목)
    └── [0-599]: {...각 프레임...}
```

---

### Frontend: Web Dashboard
**기술 스택:**
- HTML5 + CSS3 (다크테마)
- Vanilla JavaScript (모듈 없음)
- Chart.js 3.9.1 (그래프)
- Firebase REST API (폴링 기반)

**기능:**
| 기능 | 설명 |
|------|------|
| 🟢 Connection Status | Firebase 연결 상태 |
| 📊 Task Status | 현재 작업 상태 표시 |
| 📍 Position | TCP 위치 (X, Y, Z) |
| 📈 Force Graph | Z축 힘 추이 (실시간) |
| 🕐 Operational Time | 누적 운영 시간 |
| 🚨 Emergency Controls | 긴급정지/복구 버튼 |
| 📥 Log & History | 60초 히스토리 로드 |
| 📊 Data Grid | 모든 현재 값 테이블 |

**데이터 업데이트:**
- 방식: **Polling** (REST API + 500ms 주기)
- 장점: 모듈 시스템 불필요, 간단하고 신뢰할 수 있음
- 성능: ~2ms 응답시간, 실시간성 충분

---

## 📊 Data Flow

```
1. GENERATION (10Hz)
   robot_simulator.py
   └─→ ROS2 topics (5개)

2. COLLECTION & BUFFERING (10Hz)
   robot_monitor_firebase_node.py
   ├─ Subscribe 5 topics
   ├─ Buffer 600 frames (60sec)
   └─→ Firebase

3. PERSISTENCE (5Hz 현재값, 6sec 로그)
   Firebase Database
   ├─ /one (current)
   └─ /one/logs (history)

4. RETRIEVAL (500ms polling)
   firebase-rest.js
   └─ Fetch /one.json

5. DISPLAY
   Web Dashboard
   ├─ Status box
   ├─ Position panel
   ├─ Force graph
   ├─ Data grid
   └─ Update 500ms
```

---

## ⚙️ 기술 상세

### Firebase 설정
- **프로젝트**: rokey-b-3
- **Database URL**: `https://rokey-b-3-default-rtdb.firebaseio.com`
- **인증**: 서비스계정 (백엔드) + API Key (프론트엔드)
- **보안규칙**: 읽기/쓰기 public (개발 환경)

### ROS2 설정
- **Distro**: Humble 또는 Iron
- **Python**: 3.10+
- **주요 패키지**: rclpy, geometry_msgs, std_msgs, sensor_msgs
- **빌드 시스템**: colcon

### 웹서버
- **서버**: Python `http.server` (포트 8001)
- **정적 파일**: HTML, CSS, JS
- **CORS**: 로컬호스트 정책 (자동 허용)

---

## 🐛 문제 해결

### 1. "Connection Refused" on Firebase
```bash
# Firebase 인증 확인
curl 'https://rokey-b-3-default-rtdb.firebaseio.com/one.json?auth=YOUR_API_KEY'
```

### 2. ROS2 nodes 실행 안 됨
```bash
# 패키지 빌드 확인
cd ~/cobot_ws
colcon build --packages-select project_hit
source install/setup.bash

# 설치 확인
ros2 pkg list | grep project_hit
```

### 3. 웹대시보드가 데이터 안 받음
```bash
# Console (F12)에서 다음 확인:
# ✅ "Dashboard Initialization Started"
# ✅ "Chart initialized"
# ✅ "Firebase listener started (polling mode)"
# ✅ "Firebase data received (polling): ..."
```

### 4. 시뮬레이터가 느림
```bash
# Log 확인
ros2 run project_hit robot_simulator --ros-args --log-level robot_simulator:=DEBUG
```

---

## 📝 실행 체크리스트

- [ ] ROS2 환경 설정 (`source install/setup.bash`)
- [ ] `robot_simulator` 실행 중 (Terminal 1)
- [ ] `robot_monitor_firebase_node` 실행 중 (Terminal 2)
- [ ] 웹서버 실행 중 (Terminal 3)
- [ ] 브라우저에서 `http://localhost:8001/index.html` 열음
- [ ] 대시보드 상태: 🟢 Connected
- [ ] 데이터 업데이트: 5초 동안 값 변화 확인
- [ ] Force graph: 실시간 그래프 업데이트 확인

---

## 📚 상세 문서

더 자세한 기술 정보는 [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) 참고

## 📄 라이선스

Project H.I.T - Robot Monitoring System
