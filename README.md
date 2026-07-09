# KMU26 Mission FSM

Focused ROS 2 package for the KMU26 underwater mission controller.

## Nodes

- `ground_truth_buoy_fsm`: mission FSM and RC/manual/direct command output.
- `pinger_homing_controller`: hydrophone direction + optional YOLO final-align controller.
- `mission_rviz_visualizer`: RViz marker publisher for FSM state, course boundary, target state, and YOLO view.

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
  --state-topic /mavros/state
```

`FAIL` on `/odometry/filtered` or `/mavros/state` means the robot localization
or MAVROS bringup is not visible in the current ROS graph, or the topic names
do not match the launch arguments. The preflight prints similar candidate
topics when it can find them.

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

## Main Topics

- Pose input: `/odometry/filtered` (`nav_msgs/Odometry`)
- Arm state: `/mavros/state`
- RC output: `/mavros/rc/override`
- YOLO status: `/uuv_mujoco/yolo_buoy_detections`
- Hydrophone direction: `/mujoco/hydrophone/direction`
- Hydrophone status: `/mujoco/hydrophone/status`
- RViz markers: `/mission/rviz_markers`
- Mission status JSON: `/tmp/kmu26_mission_fsm_status.json`

The bundled `config/tank_current_scene.xml` is used for target layout parsing and unit checks. Real mission operation should keep live perception/localization inputs fresh and should be tested in `dry_run` before enabling command output.
