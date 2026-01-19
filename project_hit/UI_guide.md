[Role Definition] 너는 ROS2 기반의 로봇 제어 시스템을 위한 Full-stack Web Dashboard 개발자야. 아래의 요구사항 명세(Requirements)를 바탕으로 전체 시스템의 아키텍처와 주요 코드 구조를 잡아줘.

[Project Goal] ROS2 로봇암(Robot Arm)의 상태를 실시간으로 DB에 저장하고, 웹 인터페이스를 통해 모니터링 및 비상 제어를 할 수 있는 대시보드 개발.

[Core Data Flows & Features]

1. Data Pipeline (ROS2 to DB)

Input: 로봇(AP)에서 ROS2 메시지로 다음 데이터를 발행함.

로봇 작업 상태 (On/Off, 충돌, 시스템 오류, 작업 프로세스 단계)

로봇 구동 정보 (가동 시간, 좌표, 선속도, 각속도, Base/End-effector 정보)

시스템 정보 (시스템명, 버전, 기기명)

Action: 백엔드 서버는 이 데이터를 구독(Subscribe)하여 웹 DB에 실시간으로 저장해야 함.

2. Frontend Dashboard (Monitoring UI)

General: 웹 접속 시 대시보드 UI 로드.

Status Panel:

로봇 작업 여부 및 상태 표시 (Status Indicator)

작업 프로세스 및 가동 시간 표시

시스템/기기 정보 텍스트 표시

Visualization:

DB에 저장된 좌표 데이터를 기반으로 로봇암의 현재 모양(Pose)을 시각화 (Digital Twin 형태)

Data Grid: DB에 저장된 데이터를 테이블 형태로 출력하는 화면 필요.

3. Control & Interaction (User Action)

Safety Control:

[긴급정지] 및 [비상복구] 버튼 구현 (누를 시 즉시 로봇 제어 명령 전송)

Log & History:

[로그 출력] 버튼: 시스템 로그 조회

[Replay] 버튼: 직전 60초간의 작업 내용을 10Hz 단위(총 600개 프레임)로 불러와서 확인하거나 출력하는 기능

[Task] 위 3가지 흐름을 구현하기 위한 **프로젝트 파일 구조(File Structure)**와 기술 스택(Tech Stack), 그리고 핵심 컴포넌트 설계를 제안해줘.