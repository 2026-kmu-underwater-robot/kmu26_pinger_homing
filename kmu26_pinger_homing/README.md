# KMU26 Pinger Homing

실물 핑거 근방까지 자율 접근하는 전용 ROS 2 패키지다. 하이드로폰 추정 알고리즘은
저장소 최상위 `hydrophone.repos`에 고정된 별도 팀 Git 포크에서 가져오며 이 패키지는
알고리즘을 복제하지 않는다.
비전/부표 FSM과 `CollectorState`는 사용하지 않는다.

구성:

- forked `audio_capture/audio_phase_estimator`
- canonical C++ Phase/SNR RC controller (`pinger_homing_controller`)
- moving-sensor source fit and no-odometry ABBA/Huber Phase bearing mode
- exclusive `/mavros/rc/override` mux
- Pinger Homing Web GUI

실물 토픽 계약:

- 위치: `/odometry/filtered`
- 차량 상태: `/mavros/state`
- 최종 RC: `/mavros/rc/override`
- 핑거 RC mux 입력: `/control/pinger/rc_override`
- 조이스틱 RC mux 입력: `/control/joystick/rc_override`

```bash
vcs import src < src/kmu26_mission_fsm/hydrophone.repos
colcon build --symlink-install --packages-up-to kmu26_pinger_homing
source install/setup.bash
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py dry_run:=true tank_max_depth_m:=2.0
```

GUI:

```bash
ros2 run kmu26_pinger_homing start_pinger_homing_gui.sh
```

기본 주소는 `http://<robot-ip>:8878/`이다.

실물 사용 순서:

1. 프로펠러를 제거한 상태에서 먼저 `Start Robot Stack`과 `Start Dry Run`을 검증한다.
2. GUI에서 차량 모드를 설정하고 `ARM`한 뒤 `/mavros/state`가 실제 armed인지 확인한다.
3. `Preflight`가 모두 통과한 경우에만 `Start Live RC`를 누른다.
4. 조이스틱을 움직이면 조이스틱이 mux 우선권을 가져간다. `Stop` 또는 `DISARM`으로 즉시 중단한다.

GUI와 launch는 자동 아밍하지 않는다. odometry, audio, MAVROS state가 stale이거나 disarm이면
모든 RC 채널을 release한다. 기본 180초 제한 또는 추정 핑거 위치 반경 1.5 m 안에서 1초 유지 시
접근을 종료하고 RC를 release한다. 실물 IQ 거리 보정 전에는
`amplitude_range_constant=0`, `success_range_m=0`을 유지한다.

## 통합 2-D Phase/SNR 모드

`pinger_homing_test_tank.launch.py`는 이 패키지에 포함된 주파수 선택기,
오디오 IQ/Phase 추정기, 그리고 **동일한 canonical C++ controller**를 함께
실행한다. 기본 프로파일은 `ALT_HOLD + no_odom_phase + XY-only`이다.
즉 `/odometry/filtered`나 MuJoCo ground truth를 제어 입력으로 사용하지 않고,
MAVROS IMU yaw와 알려진 RC ABBA probe의 위상 거리 변화만으로 방위를 재추정한다.
Z는 ALT_HOLD가 소유하므로 RC3/heave를 명령하지 않는다.

```bash
ros2 run kmu26_pinger_homing start_pinger_homing_test_tank.sh \
  mode:=ALT_HOLD estimator_mode:=phase \
  auto_select_top:=false dry_run:=false
```

이 wrapper는 5초 감시가 끝난 뒤 같은 터미널에서 후보 번호 또는 Hz를 받는다.
후보는 100 Hz coarse scan 뒤 마지막 PCM window에서 2 Hz 간격으로 정밀 탐색되어,
예를 들어 21164 Hz 수조 송신기를 21200 Hz로 잘못 고르는 일을 막는다. 선택 결과는
현재 실행에만 유효하며 이전 DDS 실행의 주파수 선택을 재사용하지 않는다.
`ros2 launch`에 직접 입력한 표준입력은 ROS launch 자식 노드로 전달되지 않으므로,
수동 선택에는 위 wrapper를 사용한다. SNR 모드는
`estimator_mode:=snr`로 선택한다. test-tank에서 최종 RC까지 연결할 때만
`rc_output_topic:=/mavros/rc/override`를 명시한다.

수조에서 검증된 기본 PWM은 probe `±90`, 접근 `+120` (중립 1500, span 400)이며,
`probe_pwm_delta:=`와 `approach_pwm_delta:=`로 바꿀 수 있다. 실물의 초기 시험은
반드시 낮은 PWM(예: ±15--25), `dry_run:=true`, 그리고 프로펠러 제거 상태에서 시작한다.
