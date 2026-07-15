# KMU26 Vision Mission FSM

현재 시험 중인 비전 미션 전용 ROS 2 패키지다. YOLO 검출기, bbox 기반 RC 제어기,
원본 `zetex1001/auv_buoy_vision_control`에서 가져온 비전 상태기계, 영역/포획 정책을
담당하는 observation FSM을 한 패키지에 둔다.

```bash
colcon build --symlink-install --packages-up-to kmu26_vision_mission_fsm
source install/setup.bash
ros2 launch kmu26_vision_mission_fsm mission_fsm_real.launch.py \
  use_observation_mission_fsm:=true dry_run:=true
```

YOLO 모델은 저장소에 중복 저장하지 않는다. 실행 시 `vision_model_path:=/absolute/best.pt`로
지정한다. 핑거 호밍 구현은 이 패키지에 복사하지 않고 `kmu26_pinger_homing` 노드를 실행한다.
