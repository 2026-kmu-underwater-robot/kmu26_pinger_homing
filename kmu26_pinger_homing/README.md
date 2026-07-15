# KMU26 Pinger Homing

실물 핑거 근방까지 자율 접근하는 전용 ROS 2 패키지다. 원본 하이드로폰 추정 알고리즘은
`hydrophone_fork.repos`의 고정 커밋으로 가져오며 이 패키지는 알고리즘을 복제하지 않는다.
비전/부표 FSM과 `CollectorState`는 사용하지 않는다.

구성:

- forked `audio_capture/audio_phase_estimator`
- moving-sensor 3D source fit
- active-range RC controller
- exclusive `/mavros/rc/override` mux
- Pinger Homing Web GUI

실물 토픽 계약:

- 위치: `/odometry/filtered`
- 차량 상태: `/mavros/state`
- 최종 RC: `/mavros/rc/override`
- 핑거 RC mux 입력: `/control/pinger/rc_override`
- 조이스틱 RC mux 입력: `/control/joystick/rc_override`

```bash
vcs import src < src/kmu26_mission_fsm/kmu26_pinger_homing/hydrophone_fork.repos
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
