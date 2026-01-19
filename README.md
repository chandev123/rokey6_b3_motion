ddr5_tactile_handler/
├── ddr5_tactile_handler/          # [소스 코드 폴더] (파이썬 모듈)
│   ├── __init__.py                # (빈 파일, 필수)
│   ├── tactile_sensor.py          # [감각] 힘 센서 데이터 처리
│   ├── motion_control.py          # [동작] 회피/삽입 모션 제어
│   └── main_state_machine.py      # [두뇌] 전체 순서 관리 (탐색->삽입->검증)
│
├── launch/                        # [실행 파일 폴더]
│   └── ddr5_handler.launch.py     # 로봇 + 내 코드 3개를 한방에 실행하는 파일
│
├── config/                        # [설정 파일 폴더]
│   └── ddr5_params.yaml           # 임계값(5N), 목표 좌표 등 변수 모음
│
├── package.xml                    # 패키지 정보
├── setup.py                       # 빌드 설정
└── setup.cfg
