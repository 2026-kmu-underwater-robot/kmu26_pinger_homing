# KMU26 AUV Control

이 저장소 자체가 핑거 호밍 ROS 2 패키지 `kmu26_pinger_homing`이다. 2-D Phase/SNR 주파수
선택기, 오디오 추정기, RC 제어기는 모두 이 루트 패키지에 통합되어 있다.
비전 미션 FSM은 작업공간의 `archive/kmu26_vision_mission_fsm`으로 분리되어
기본 빌드와 실행에 포함되지 않는다.

```text
package.xml                 ROS package manifest
CMakeLists.txt              ROS package build definition
launch/                     Phase/SNR/interactive launch files
src/                        C++ controller and frequency selector
```

NUC의 최종 소스 경계는 다음과 같다.

```text
~/auv_ws/src/
├── kmu26_pinger_homing/            # 이 Git 저장소 = ROS package root
│   ├── package.xml
│   ├── launch/
│   └── src/
├── kmu26_auv_hydrophone/           # 별도 Git 저장소, 신호처리 ROS 패키지들
    ├── audio_common/
    ├── audio_common_msgs/
    └── audio_capture/
└── archive/kmu26_vision_mission_fsm/  # src 밖의 보관본
```

`kmu26_auv_hydrophone`을 이 저장소 안에 복사하거나 중첩 clone하지 않는다.

## 설치

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone --branch main https://github.com/2026-kmu-underwater-robot/kmu26_auv_pinger_homing.git kmu26_pinger_homing
vcs import src < src/kmu26_pinger_homing/hydrophone.repos
git clone https://github.com/2026-kmu-underwater-robot/kmu26_auv.git
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to kmu26_pinger_homing
source install/setup.bash
```

이제 clone 직후 실제 패키지 경로는
`~/auv_ws/src/kmu26_pinger_homing` 하나뿐이다. 예전의
`~/auv_ws/src/kmu26_pinger_homing/kmu26_pinger_homing` 경로는 사용하지 않는다.

세부 Phase/FFT 판정 기준은 [PINGER_HOMING.md](PINGER_HOMING.md)를 참고한다.

`hit25_auv_ros2`가 사용하는 팀 `dvl_msgs` 패키지도 실물 ROS 작업공간에 있어야 한다.

## 실행

실물 오디오 스트림이 이미 실행 중일 때, 주파수를 스캔·선택하는 dry-run:

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true use_audio_capture:=false tank_max_depth_m:=2.0
```

launch는 5초 뒤 후보 1~5 또는 Hz를 터미널에서 받으며, 선택 주파수로
기존 hydrophone estimator를 시작한 뒤 canonical C++ no-odometry Phase
제어기를 실행한다. 실물 초기 시험은 이 interactive launch를 쓴다. 처음에는 반드시 프로펠러를 제거하고
`dry_run:=true`로 토픽과 추정 상태부터 확인한다.

## test-tank Phase/SNR 핑거 호밍

```bash
ros2 run kmu26_pinger_homing start_pinger_homing_test_tank.sh \
  mode:=ALT_HOLD estimator_mode:=phase \
  rc_output_topic:=/mavros/rc/override \
  auto_select_top:=false dry_run:=false
```

시작 후 5초 동안 주파수를 스캔해 후보를 최대 5개 표시한다. 같은 터미널에 후보 번호
(`1`~`5`) 또는 주파수(Hz)를 입력하면 2-D probe, yaw 정렬, 전진 호밍이 시작된다.
`estimator_mode:=snr`로 SNR 모드를 선택할 수 있다. 다른 RC 소유권을 보존하려면
`rc_output_topic`을 기본 `/control/pinger/rc_override`로 둔다.

비전 미션 FSM을 다시 사용할 때는 `archive/kmu26_vision_mission_fsm`을 별도 ROS
작업공간으로 옮겨 독립적으로 빌드한다.
