# KMU26 Pinger Homing

완성된 실물 핑거 호밍 전용 ROS 2 패키지다. 원본 하이드로폰 추정 알고리즘은
`hydrophone_fork.repos`의 고정 커밋으로 가져오며 이 패키지는 알고리즘을 복제하지 않는다.

구성:

- forked `audio_capture/audio_phase_estimator`
- moving-sensor 3D source fit
- active-range RC controller
- exclusive `/mavros/rc/override` mux
- Pinger Homing Web GUI

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

기본 주소는 `http://<robot-ip>:8080/`이다. GUI와 launch는 자동 아밍하지 않으며,
실물 IQ 거리 보정 전에는 `amplitude_range_constant=0`, `success_range_m=0`을 유지한다.
