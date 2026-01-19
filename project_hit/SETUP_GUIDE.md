# 🛠️ Project H.I.T - Setup & Deployment Guide

**프로젝트 설정 및 배포 완벽 가이드**

---

## 📋 목차

1. [환경 구성](#1-환경-구성)
2. [초기 설정](#2-초기-설정)
3. [실행 방법](#3-실행-방법)
4. [배포](#4-배포)
5. [모니터링](#5-모니터링)

---

## 1. 환경 구성

### 1.1 필수 요구사항

```bash
# OS
Ubuntu 20.04 LTS or later

# ROS2
- Humble 또는 Iron
- colcon build system

# Python
3.10 or higher

# Node.js (선택사항)
npm packages 없음

# 네트워크
- 인터넷 연결 (Firebase)
- localhost 포트 8001 사용 가능
```

### 1.2 패키지 설치

```bash
# ROS2 기본 패키지 설치
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs

# Python 의존성
cd ~/cobot_ws/src/project_hit
pip3 install -r requirements.txt 2>/dev/null || pip3 install firebase-admin
```

### 1.3 Firebase 설정

#### 이미 구성됨 ✅
- 프로젝트: `rokey-b-3`
- Database URL: `https://rokey-b-3-default-rtdb.firebaseio.com`
- Service Account: `rokey-b-3-firebase-adminsdk-fbsvc-09612845da.json` (커밋됨)
- API Key: `AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48` (프론트엔드용)

#### Firebase CLI (선택사항 - 고급)
```bash
# Firebase CLI 설치
npm install -g firebase-tools

# 인증
firebase login

# 프로젝트 초기화
firebase init

# 배포
firebase deploy
```

---

## 2. 초기 설정

### 2.1 워크스페이스 준비

```bash
# 홈디렉토리에서 작업
cd ~

# 기존 빌드 정리 (선택사항)
rm -rf cobot_ws/build cobot_ws/install

# ROS2 환경 설정
cd cobot_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### 2.2 패키지 빌드

```bash
# 전체 패키지 빌드
colcon build

# 또는 프로젝트만 빌드
colcon build --packages-select project_hit

# 빌드 후 설정
source install/setup.bash

# 확인
ros2 pkg list | grep project_hit
```

### 2.3 환경 변수 설정

```bash
# ~/.bashrc 에 추가 (선택사항)
echo "source ~/cobot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. 실행 방법

### 3.1 기본 3단계 실행

#### Terminal 1: 로봇 시뮬레이터
```bash
cd ~/cobot_ws
source install/setup.bash

ros2 run project_hit robot_simulator
```

**예상 출력:**
```
[INFO] [1768812500.123456789] robot_simulator: 
🤖 Robot Simulator Started
📊 Publishing at 10Hz
Topics:
  - /dsr01/task_status
  - /dsr01/force_torque_data
  - /dsr01/robot_pose
  - /dsr01/system_info
  - /dsr01/operational_time
```

#### Terminal 2: Firebase 모니터 노드
```bash
cd ~/cobot_ws
source install/setup.bash

ros2 run project_hit robot_monitor_firebase_node
```

**예상 출력:**
```
[INFO] [1768812505.234567890] robot_monitor_firebase_node:
🚀 Robot Monitor Firebase Node Started
🔗 Connecting to Firebase...
✅ Firebase initialized successfully

[INFO] [1768812505.345678901] ✓ Status: WAITING | Fz: 5.23N | Pos: (250.0, 400.0, 230.0)
[INFO] [1768812505.845678902] ✓ Status: INSERTING | Fz: 12.3N | Pos: (300.0, 400.0, 250.0)
```

#### Terminal 3: 웹서버
```bash
cd ~/cobot_ws/src/project_hit/frontend
python3 -m http.server 8001
```

**예상 출력:**
```
Serving HTTP on 0.0.0.0 port 8001 (http://0.0.0.0:8001/) ...
127.0.0.1 - - [19/Jan/2026 15:00:00] "GET /index.html HTTP/1.1" 200 -
127.0.0.1 - - [19/Jan/2026 15:00:00] "GET /css/dashboard.css HTTP/1.1" 200 -
127.0.0.1 - - [19/Jan/2026 15:00:00] "GET /js/firebase-rest.js HTTP/1.1" 200 -
```

### 3.2 웹대시보드 접근

```bash
# 브라우저에서 열기
http://localhost:8001/index.html
```

**체크포인트:**
- [ ] 페이지 로드됨
- [ ] 🟢 Connection status: **Connected** (1-2초 내)
- [ ] 데이터 업데이트 (500ms마다)
- [ ] 그래프 움직임
- [ ] 상태 변화 (WAITING → INSERTING → ...)

---

## 4. 배포

### 4.1 성능 모드 실행

```bash
# 각 터미널에서 로깅 레벨 조정
ros2 run project_hit robot_simulator \
  --ros-args --log-level WARN

ros2 run project_hit robot_monitor_firebase_node \
  --ros-args --log-level INFO
```

### 4.2 백그라운드 실행 (선택사항)

```bash
# Screen 또는 tmux 사용
screen -S robot_system

# 또는 systemd service로 등록
sudo systemctl enable --now project_hit.service
```

### 4.3 Docker 배포 (고급)

```dockerfile
# Dockerfile 예제
FROM osrf/ros:humble-desktop

WORKDIR /cobot_ws

# 의존성 설치
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install firebase-admin

# 소스 복사
COPY . .

# 빌드
RUN colcon build

# 실행
CMD ["bash", "-c", "source install/setup.bash && ros2 run project_hit robot_monitor_firebase_node"]
```

**빌드 & 실행:**
```bash
docker build -t robot-monitor .
docker run -p 8001:8001 robot-monitor
```

---

## 5. 모니터링

### 5.1 실시간 로그 확인

```bash
# 현재 로그 모니터링
tail -f /tmp/monitor.log | grep "Status\|Fz"

# 또는
ros2 run project_hit robot_monitor_firebase_node 2>&1 | tee /tmp/monitor.log
```

### 5.2 Firebase 데이터 확인

```bash
# REST API로 데이터 조회
curl 'https://rokey-b-3-default-rtdb.firebaseio.com/one.json?auth=AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48' | jq .

# 특정 필드만 조회
curl 'https://rokey-b-3-default-rtdb.firebaseio.com/one/status.json?auth=AIzaSyCVaEaIp1lyqLlvKR7rBFDpLNyp3Iavx48'
```

### 5.3 웹대시보드 콘솔 로그

브라우저에서 F12 → Console 탭:

```javascript
// 현재 상태 확인
console.log(document.getElementById('status-box').textContent);

// 연결 상태 확인
console.log(document.getElementById('connection-status').textContent);

// 마지막 업데이트 시간 확인
console.log(document.getElementById('grid-timestamp').textContent);
```

### 5.4 ROS2 토픽 모니터링

```bash
# 발행 토픽 확인
ros2 topic list

# 토픽 데이터 확인
ros2 topic echo /dsr01/task_status
ros2 topic echo /dsr01/force_torque_data
ros2 topic echo /dsr01/robot_pose

# 퍼블리셔 정보
ros2 node info /robot_simulator
```

---

## 🐛 문제 해결

### Issue 1: "ModuleNotFoundError: No module named 'firebase_admin'"

```bash
pip3 install firebase-admin
# 또는
pip3 install -r requirements.txt
```

### Issue 2: "ROS2 command not found"

```bash
# ROS2 환경 설정 확인
echo $ROS_DISTRO

# 설정되지 않으면
source /opt/ros/humble/setup.bash
source ~/cobot_ws/install/setup.bash
```

### Issue 3: "Connection refused" on Firebase

```bash
# 인터넷 연결 확인
ping google.com

# Firebase URL 접근 확인
curl -I https://rokey-b-3-default-rtdb.firebaseio.com

# API Key 유효성 확인
curl 'https://rokey-b-3-default-rtdb.firebaseio.com/one.json?auth=YOUR_API_KEY'
```

### Issue 4: 웹대시보드가 데이터 안 받음

**Console (F12) 확인:**
```
✅ "Dashboard Initialization Started"
✅ "Chart initialized"  
✅ "Firebase listener started (polling mode)"
✅ "Firebase data received (polling): ..."
```

**해결 방법:**
1. 로봇 시뮬레이터 실행 확인
2. Monitor 노드 실행 확인
3. Firebase 연결 확인 (curl 테스트)
4. 브라우저 캐시 삭제 (Ctrl+Shift+Delete)
5. 페이지 새로고침 (F5)

### Issue 5: 포트 8001 이미 사용 중

```bash
# 프로세스 확인
lsof -i :8001

# 또는 다른 포트 사용
python3 -m http.server 8002
# 브라우저: http://localhost:8002/index.html
```

---

## 📊 성능 최적화

### 메모리 사용
```bash
# 프로세스 메모리 확인
ps aux | grep -E "robot_simulator|robot_monitor"

# 또는
top -p $(pgrep -f robot_simulator)
```

### CPU 사용
```bash
# CPU 활용률 확인
htop -p $(pgrep -f robot_monitor_firebase_node)
```

### 네트워크 대역폭
```bash
# Firebase 업로드 속도 확인
# Monitor node 로그에서 timestamp 비교
# 평상시: ~200ms 간격 (5Hz)
```

---

## 🔒 보안 고려사항

### 개발 환경
- Firebase 보안규칙: 공개 (개발용)
- API Key: 코드에 포함 (프론트엔드)
- 서비스계정: git에 포함 (내부용)

### 프로덕션 환경
```json
{
  "rules": {
    ".read": "auth != null",
    ".write": "auth != null"
  }
}
```

```bash
# 민감한 파일 보호
git rm --cached rokey-b-3-firebase-adminsdk-fbsvc-*.json
echo "rokey-b-3-firebase-adminsdk-fbsvc-*.json" >> .gitignore
```

---

## 📚 참고 문서

- [README.md](README.md) - 프로젝트 개요
- [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md) - 기술 상세
- [ROS2 공식 문서](https://docs.ros.org/)
- [Firebase 문서](https://firebase.google.com/docs)

---

**Last Updated:** 2026-01-19
**Version:** 1.0
