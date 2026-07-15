# KMU26 Mission FSM

Focused ROS 2 package for the KMU26 underwater mission controller.

## Nodes

- `ground_truth_buoy_fsm`: mission FSM and RC/manual/direct command output.
- `pinger_homing_controller`: hydrophone direction + optional YOLO final-align controller.
- `single_hydrophone_homing_controller.py`: moving-sensor active-range pinger localizer and real-vehicle RC controller.
- `rc_override_mux`: exclusive owner of the final `/mavros/rc/override` output.
- `mission_rviz_visualizer`: RViz marker publisher for FSM state, course boundary, target state, and YOLO view.
- `fsm_web_gui.py`: dedicated FSM web GUI for mission start/stop, RViz helpers, camera preview, state, RC monitor, and course boundary setup.

## Build

Place this repository under a ROS 2 workspace `src/` directory, then:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kmu26_mission_fsm
source install/setup.bash
```

After pulling updates on the vehicle NUC, run the preflight from the workspace
without permanently changing the shell environment:

```bash
./src/kmu26_mission_fsm/scripts/real_vehicle_preflight.sh --build
```

If the vehicle uses different topic names, pass them explicitly:

```bash
./src/kmu26_mission_fsm/scripts/real_vehicle_preflight.sh --build \
  --pose-topic /odometry/filtered \
  --state-topic /mavros/state \
  --dvl-twist-topic /dvl/twist \
  --depth-topic /depth/pose \
  --camera-compressed-topic /camera/camera/color/image_raw/compressed \
  --camera-raw-topic /camera/camera/color/image_raw
```

The preflight follows the original `kmu26_auv_web_gui` real-robot contract:
`hit25_auv_ros2 localization_test.launch.py` is the robot stack, and the GUI
expects `/odometry/filtered`, `/dvl/twist`, `/depth/pose`,
`/mavros/imu/data`, `/joy`, `/battery`,
`/camera/camera/color/image_raw/compressed`, and
`/camera/camera/color/image_raw`.

`FAIL` on `/odometry/filtered` or `/mavros/state` means the robot localization
or MAVROS bringup is not visible in the current ROS graph, or the topic names do
not match the FSM launch arguments. `WARN` on DVL/depth/IMU/joy/battery means
the mission package can still parse and start, but the real GUI stack is not
publishing the same status topics it normally uses. The preflight prints similar
candidate topics when it can find them.

After the package is built, the same check is also available through ROS:

```bash
ros2 run kmu26_mission_fsm real_vehicle_preflight.sh
```

## Real-Vehicle Bringup

The focused launch is conservative by default. It starts the headless RViz marker visualizer, but does not start RViz or autonomous control unless explicitly enabled. This avoids Qt/X11 display failures when running on the vehicle NUC through Docker or SSH.

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py
```

Start RViz only when running on a machine with a working display:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_rviz:=true
```

Start the FSM in dry-run mode:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_mission_fsm:=true dry_run:=true
```

`dry_run` disables command output only. The FSM still needs vehicle pose for state
transitions, relative target coordinates, and RViz/status output. The real-vehicle
launch defaults to `pose_topic:=/odometry/filtered pose_type:=odometry`.

Start the FSM with RC output after MAVROS is ready and the safety path has been checked:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_mission_fsm:=true dry_run:=false wait_armed:=true transport:=rc_override
```

Start pinger-only homing:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_pinger_homing:=true
```

For the validated active-range controller on a real vehicle, use the dedicated
launch. Start disarmed in dry-run mode and provide the actual pool depth:

```bash
ros2 launch kmu26_mission_fsm pinger_homing_real.launch.py \
  dry_run:=true use_audio_capture:=false \
  use_hydrophone_estimator:=true tank_max_depth_m:=2.0 \
  amplitude_range_constant:=0.0 success_range_m:=0.0
```

The controller consumes `/odometry/filtered`, `/mavros/state`,
`/audio_phase_estimator/delta_range_m`, `/audio_phase_estimator/iq_magnitude`,
and `/homing/direction`. It publishes to `/control/pinger/rc_override`; the
dedicated mux is the only node allowed to publish `/mavros/rc/override`.
Dry-run always publishes `CHAN_RELEASE`. Live mode never arms the vehicle and
only applies RC while MAVROS reports `armed=true`. Keep the IQ range constant
and success range at zero until the physical hydrophone has been calibrated.

## Dedicated FSM GUI

This is separate from the MuJoCo simulator GUI and from `kmu26_auv_web_gui`.
It is meant to operate the mission package directly.

```bash
ros2 launch kmu26_mission_fsm mission_fsm_gui.launch.py camera_on:=true
```

Open:

```text
http://127.0.0.1:8890/
```

For a remote laptop, bind to the NUC network interface:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_gui.launch.py host:=0.0.0.0 camera_on:=true
```

The GUI can start/stop the mission FSM, pinger homing, RViz marker visualizer,
and RViz. It also streams a camera from the real-vehicle compressed topic
`/camera/camera/color/image_raw/compressed` or the raw fallback
`/camera/camera/color/image_raw`, reads
`/tmp/kmu26_mission_fsm_status.json`, shows topic health, and stores course
boundary settings in
`/tmp/kmu26_mission_fsm_gui_config.json`.

If OpenCV/NumPy raw conversion is unavailable on the NUC, the compressed camera
path still works and the GUI reports the raw conversion error in `/api/status`.

RC publishing is locked by default. Enable it only on a checked bench/safety
path:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_gui.launch.py allow_rc_send:=true
```

## Main Topics

- Pose input: `/odometry/filtered` (`nav_msgs/Odometry`)
- Arm state: `/mavros/state`
- GUI DVL velocity: `/dvl/twist` (`geometry_msgs/TwistWithCovarianceStamped`)
- GUI depth pose: `/depth/pose` (`geometry_msgs/PoseWithCovarianceStamped`)
- GUI IMU: `/mavros/imu/data` (`sensor_msgs/Imu`)
- GUI joystick: `/joy` (`sensor_msgs/Joy`)
- GUI battery: `/battery` (`sensor_msgs/BatteryState`)
- RC output: `/mavros/rc/override`
- YOLO status: `/uuv_mujoco/yolo_buoy_detections`
- Hydrophone direction: `/mujoco/hydrophone/direction`
- Hydrophone status: `/mujoco/hydrophone/status`
- RViz markers: `/mission/rviz_markers`
- Mission status JSON: `/tmp/kmu26_mission_fsm_status.json`

The bundled `config/tank_current_scene.xml` is used for target layout parsing and unit checks. Real mission operation should keep live perception/localization inputs fresh and should be tested in `dry_run` before enabling command output.
