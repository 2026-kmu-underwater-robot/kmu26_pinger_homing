# KMU26 AUV Control

이 저장소는 핑거 호밍에 필요한 ROS 2 패키지만 관리한다. `kmu26_finger_homing`은
하이드로폰 fork나 FSM에 의존하지 않는 독립 2-D Phase/SNR 핑거 호밍 패키지다.
비전 미션 FSM은 작업공간의 `archive/kmu26_vision_mission_fsm`으로 분리되어
기본 빌드와 실행에 포함되지 않는다.

```text
kmu26_pinger_homing/       완성된 하이드로폰 핑거 호밍 + RC + Web GUI
kmu26_finger_homing/       독립 2-D Phase/SNR 호밍 + 5초 주파수 선택
```

NUC의 최종 소스 경계는 다음과 같다.

```text
~/auv_ws/src/
├── kmu26_pinger_homing/            # 이 Git 저장소
│   ├── kmu26_pinger_homing/        # 차량측 RC 제어·mux·GUI ROS 패키지
│   ├── kmu26_finger_homing/        # 독립 2-D Phase/SNR 패키지
│   └── (archive는 ../archive/kmu26_vision_mission_fsm에 별도 보관)
└── kmu26_auv_hydrophone/           # 별도 Git 저장소, 신호처리 ROS 패키지들
    ├── audio_common/
    ├── audio_common_msgs/
    └── audio_capture/
```

`kmu26_auv_hydrophone`을 이 저장소 안에 복사하거나 중첩 clone하지 않는다.

## 설치

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone --branch main https://github.com/2026-kmu-underwater-robot/kmu26_pinger_homing.git
git clone https://github.com/2026-kmu-underwater-robot/kmu26_auv.git
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to kmu26_finger_homing kmu26_pinger_homing
source install/setup.bash
```

`hit25_auv_ros2`가 사용하는 팀 `dvl_msgs` 패키지도 실물 ROS 작업공간에 있어야 한다.

## 실행

완성된 핑거 호밍 dry-run:

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  dry_run:=true tank_max_depth_m:=2.0
```

핑거 Web GUI:

```bash
ros2 run kmu26_pinger_homing start_pinger_homing_gui.sh
```

`http://<robot-ip>:8878/`에서 `Start Robot Stack` → mode 설정 → `ARM` → `Preflight` →
`Start Live RC` 순서로 실행한다. 처음에는 반드시 프로펠러를 제거하고 `Start Dry Run`으로
토픽과 추정 상태부터 확인한다.

## 독립 test-tank 핑거 호밍

```bash
ros2 launch kmu26_finger_homing finger_homing_test_tank.launch.py \
  mode:=ALT_HOLD estimator_mode:=phase \
  rc_output_topic:=/mavros/rc/override \
  auto_select_top:=false dry_run:=false
```

시작 후 5초 동안 주파수를 스캔해 후보를 최대 5개 표시한다. 터미널에 후보 번호
(`1`~`5`) 또는 주파수(Hz)를 입력하면 2-D probe, yaw 정렬, 전진 호밍이 시작된다.
`estimator_mode:=snr`로 SNR 모드를 선택할 수 있다. 다른 RC 소유권을 보존하려면
`rc_output_topic`을 기본 `/control/finger_homing/rc_override`로 둔다.

비전 미션 FSM을 다시 사용할 때는 `archive/kmu26_vision_mission_fsm`을 별도 ROS
작업공간으로 옮겨 독립적으로 빌드한다.
