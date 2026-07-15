# 핑거 호밍 RC 제어기 배포 구조

## 코드 경계

배포본은 기존 하이드로폰 신호처리 알고리즘과 차량 RC 제어기를 분리한다.

```text
팀 포크 kmu26_auv_hydrophone
  /audio
    -> audio_phase_estimator
    -> /audio_phase_estimator/delta_range_m
    -> /audio_phase_estimator/iq_magnitude
    -> /homing/direction

kmu26_pinger_homing/src/pinger_homing
  위 추정 결과 + /odometry/filtered + /mavros/state
    -> 능동 위치 추정 및 접근 RC
    -> /control/pinger/rc_override

kmu26_vision_mission_fsm/src/vision_buoy
  /vision/buoy_observation
    -> bbox 정렬 RC
    -> /control/mission/rc_override

rc_override_mux
  -> /mavros/rc/override
```

하이드로폰 의존성은 `hydrophone_fork.repos`에 공개 upstream URL과 검증 커밋
`64ccbadad903b6d5b40f641996ce6fe91fd1f69d`로 고정한다. 하이드로폰 추정 알고리즘을
FSM 안으로 복사하거나 수정하지 않는다.

## 기존 알고리즘에 추가한 차량 제어 코드

### 1. 능동 3차원 탐색

`single_hydrophone_homing_controller.py`가 다음 순서로 RC 탐색을 수행한다.

```text
WAIT_VEHICLE -> PROBE <-> REPROBE -> ALIGN <-> APPROACH -> CONTACT -> COMPLETE
```

한 위치의 단일 하이드로폰 측정만으로는 3차원 방향을 결정하기 어려우므로, 전진·횡이동·수직
탐색 구간에서 odometry 위치와 음향 거리 변화를 함께 수집한다. 각 구간 사이에는 중립 RC를
넣어 관성 운동의 영향을 줄인다.

### 2. 움직이는 단일 센서 위치 추정

`single_hydrophone_homing_math.py`가 다음 정보를 결합한다.

- 위상 변화 기반 상대 거리 변화
- coherent-IQ 크기 기반 절대 거리 보조값
- 각 측정 시점의 차량 3차원 위치
- 주파수/클럭 편차를 흡수하는 거리 변화율 bias

표본 수, 잔차, 조건수가 기준을 통과할 때만 추정 위치를 고정한다. 품질이 부족하면 접근하지
않고 반대 방향으로 `REPROBE`한다.

### 3. RC 생성과 안전 처리

- 핑거 월드 위치 벡터를 차량 body FLU 좌표로 변환
- yaw 오차에 비례 제어와 yaw-rate 감쇠 적용
- 큰 방위 오차에서는 전진 제한, 정렬 후 거리별 단계 감속
- `tank_max_depth_m` 하나로 차량 최대 깊이와 수직 탐색 방향 자동 계산
- 바닥 제한에 가까우면 하강 명령 차단 및 상승 복구
- 실물 live 모드에서 odometry, audio 또는 arm 상태가 유효하지 않으면 모든 RC 채널 release
- dry-run에서는 탐색/추정/요청 명령을 그대로 계산하되 항상 모든 RC 채널 release
- 상태와 명령을 `/pinger_homing/status` JSON으로 공개

실물 출력은 RC mux 입력 `/control/pinger/rc_override`로 보내며, mux만 최종
`/mavros/rc/override`를 소유한다.

## 비전 부표 알고리즘 분리

YOLO 추론은 별도 패키지 `kmu26_auv_buoy_vision_control`이 담당한다. FSM은 typed bbox 결과만
받는다. `src/vision_buoy/vision_guidance.hpp`에는 bbox 중심 오차, 오차 변화율, bbox 크기를
forward/yaw/heave 명령으로 변환하는 ROS 비의존 제어식만 둔다.

- 수중 일반 부표: `stick` 우선 정렬
- 핑거 근접 단계: `buoy` 또는 `stick`으로 최종 정렬
- 수면 빨간 부표: 포획망 중앙 정렬
- odometry와 영역 경계를 이용해 우리 영역 밖 표적 제외

## 빌드 및 실행

```bash
vcs import src < src/kmu26_mission_fsm/kmu26_pinger_homing/hydrophone_fork.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  audio_common_msgs audio_capture hit25_auv_ros2_msg \
  kmu26_pinger_homing kmu26_vision_mission_fsm
source install/setup.bash
```

핑거 호밍만 실행:

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  dry_run:=true \
  use_audio_capture:=false \
  use_hydrophone_estimator:=true \
  tank_max_depth_m:=11.0
```

`kmu26_pinger_homing`의 **Pinger Homing** Web GUI도 같은 전용 launch를 실행한다. GUI의 Live
RC 버튼은 자동 아밍하지 않고, `/mavros/state`가 armed일 때만 컨트롤러 출력이 mux를 통해
`/mavros/rc/override`로 전달된다. 실물 IQ-거리 보정 전에는
`amplitude_range_constant:=0.0`, `success_range_m:=0.0`을 유지한다.

비교용 직접 방향 추종 제어기는 명시적으로 `pinger_controller:=direction`을 선택할 때만 사용한다.
기본값은 검증 성공률이 가장 높았던 `active_range`이다.
