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

실물 토픽 계약:

- 위치: `/odometry/filtered`
- 차량 상태: `/mavros/state`
- 최종 RC: `/mavros/rc/override`
- 핑거 RC mux 입력: `/control/pinger/rc_override`
- 조이스틱 RC mux 입력: `/control/joystick/rc_override`

```bash
vcs import src < src/kmu26_pinger_homing/hydrophone.repos
colcon build --symlink-install --packages-up-to kmu26_pinger_homing
source install/setup.bash
ros2 launch kmu26_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true use_audio_capture:=false tank_max_depth_m:=2.0
```

실물 사용 순서:

1. 프로펠러를 제거한 상태에서 interactive launch의 `dry_run:=true`를 먼저 검증한다.
2. MAVROS에서 ALT_HOLD와 arm 상태를 확인한다.
3. 토픽과 Phase 추정이 정상일 때만 `dry_run:=false`를 사용한다.
4. 조이스틱 `/control/joystick/rc_override`는 mux 우선권을 가져간다. launch 종료 또는 DISARM으로 즉시 중단한다.

실물 wrapper는 시작 전에 5초 주파수 스캔을 수행하고 후보 1~5 또는 Hz를
같은 터미널에서 받는다. 선택한 주파수로 기존 `audio_phase_estimator`를
시작한 뒤, 검증된 C++ no-odometry Phase 제어기를 실행한다.

C++ launch는 자동 아밍하지 않는다. audio, MAVROS state 또는 IMU가
stale이거나 disarm/ALT_HOLD 이탈이면 모든 RC 채널을 release한다. 실물 IQ
거리 보정 전에는 `amplitude_range_constant=0`, `success_range_m=0`을 유지한다.

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
후보는 기존 `kmu26_auv_hydrophone` FFT 노드의 계약을 따라, 8192-sample Hann FFT
(96 kHz 입력이면 **11.72 Hz/bin**), 50% overlap, 5초 Welch 평균 스펙트럼, band median
noise-floor SNR, local peak prominence, 그리고 반복 검출 횟수로 판정한다. 따라서 한
프레임 노이즈나 강한 side-lobe는 후보가 되지 않고, 수조 송신기 21164 Hz도 FFT peak
interpolation으로 100 Hz 단위로 반올림하지 않는다. 기본 품질 기준은 `SNR >= 9 dB`,
prominence `>= 4.5 dB`, 반복 FFT window `>= 4`회다. `scan_fft_size`,
`scan_min_snr_db`, `scan_min_peak_prominence_db`, `scan_minimum_candidate_hits` launch
파라미터로 수조/실물 잡음 환경에 맞출 수 있다. 또 최고 후보보다 기본 18 dB 이상 약한
고정 스펙트럼 피크는 `scan_relative_to_top_snr_db`로 목록에서 제외한다(모든 후보를
보고 싶으면 `:=0`). 선택 결과는 현재 실행에만 유효하며
이전 DDS 실행의 주파수 선택을 재사용하지 않는다.
`ros2 launch`에 직접 입력한 표준입력은 ROS launch 자식 노드로 전달되지 않으므로,
수동 선택에는 위 wrapper를 사용한다. SNR 모드는
`estimator_mode:=snr`로 선택한다. test-tank에서 최종 RC까지 연결할 때만
`rc_output_topic:=/mavros/rc/override`를 명시한다.

`pinger_homing_test_tank.launch.py`의 기본 Phase 프로파일은 probe `±90`, 접근 `+120`
(중립 1500, span 400), leg `1.25 s`, neutral `0.35 s`, settle `0.55 s`, sample delay
`0.40 s`, 초기 확인 2회, 접근/reprobe `3.0 s`, RC loop `30 Hz`다. 이 값은 빠른
0.4초 probe가 controller의 0.5초 하한으로 잘리고 안정 구간을 잃는 문제를 피한다.
`probe_pwm_delta:=`와 `approach_pwm_delta:=`로 바꿀 수 있다. 실물의 초기 시험은
반드시 낮은 PWM(예: ±15--25), `dry_run:=true`, 그리고 프로펠러 제거 상태에서 시작한다.
