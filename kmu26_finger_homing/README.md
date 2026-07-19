# KMU26 Finger Homing (standalone)

이 패키지는 외부 `hydrophone.repos`, `kmu26_auv_hydrophone`, FSM, RC mux를
실행 의존성으로 사용하지 않는 독립 핑거 호밍 패키지다.

- 오디오 demodulation/Phase delta-range와 IQ SNR을 패키지 안에서 계산한다.
- 시작 후 5초 동안 주파수를 모니터링하고 반복 검출된 상위 1~5개 후보를 터미널에 표시한다.
- 사용자가 후보 번호 또는 주파수를 입력하면 선택 주파수를 고정하고 homing을 시작한다.
- Phase와 SNR 모두 XY 평면만 사용하며 Z/수심을 구독하지 않는다.
- 차량 모드는 `ALT_HOLD`를 기본 계약으로 사용하고 heave RC는 항상 중립이다.
- RC 출력 기본값은 `/control/finger_homing/rc_override`라 다른 제어기의 최종 소유권을 건드리지 않는다.

## 실행

```bash
source install/setup.bash
ros2 launch kmu26_finger_homing finger_homing_test_tank.launch.py \
  mode:=ALT_HOLD \
  rc_output_topic:=/mavros/rc/override
```

실물에서는 기본 private RC topic을 사용해 기존 소유권 관리자와 별도로 연결한 뒤,
운영자가 승인한 별도 bridge에서 최종 RC topic으로 연결한다.

## 노드와 토픽

`finger_frequency_selector`

- 입력: `/audio`
- 출력: `/finger_homing/selected_frequency_hz`, `/finger_homing/frequency_candidates`

`finger_audio_estimator`

- 입력: `/audio`, 선택 주파수
- 출력: `/finger_homing/delta_range_m`, `/finger_homing/iq_snr_db`

`finger_homing_controller`

- 입력: `/finger_homing/delta_range_m`, `/finger_homing/iq_snr_db`, `/odometry/filtered`, `/mavros/state`
- 출력: configurable RC topic, `/finger_homing/status`, `/finger_homing/direction`

Phase는 중립 구간이 있는 `+X,-X,-X,+X,+Y,-Y,-Y,+Y` probe로 2-D range gradient를
추정한다. SNR은 최근 XY 표본의 robust 평면 회귀와 방향 EMA/일관성 hold를 결합한다.

## test-tank 검증

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
ros2 launch kmu26_finger_homing finger_homing_test_tank.launch.py \
  mode:=ALT_HOLD estimator_mode:=phase auto_select_top:=true \
  rc_output_topic:=/mavros/rc/override dry_run:=false
```

테스트 탱크에서 5초 주파수 감시 후 후보를 출력하고, phase와 SNR 모두
`PROBE_2D -> ALIGN -> APPROACH -> SUCCESS`를 확인했다. `auto_select_top:=false`로
실행하면 후보 출력 뒤 표준입력에 `1`~`5` 또는 주파수(Hz)를 입력해 선택할 수 있다.
직접 RC 소유권을 건드리지 않으려면 `rc_output_topic`을 기본값으로 유지한다.
