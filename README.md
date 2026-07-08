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

## Real-Vehicle Bringup

The focused launch is conservative by default. It starts the RViz marker visualizer and RViz, but does not start autonomous control unless explicitly enabled.

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py
```

Start the FSM in dry-run mode:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_mission_fsm:=true dry_run:=true
```

Start the FSM with RC output after MAVROS is ready and the safety path has been checked:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_mission_fsm:=true dry_run:=false wait_armed:=true transport:=rc_override
```

Start pinger-only homing:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_pinger_homing:=true
```

## Main Topics

- Pose input: `/mavros/local_position/pose`
- Arm state: `/mavros/state`
- RC output: `/mavros/rc/override`
- YOLO status: `/uuv_mujoco/yolo_buoy_detections`
- Hydrophone direction: `/mujoco/hydrophone/direction`
- Hydrophone status: `/mujoco/hydrophone/status`
- RViz markers: `/mission/rviz_markers`
- Mission status JSON: `/tmp/kmu26_mission_fsm_status.json`

The bundled `config/tank_current_scene.xml` is used for target layout parsing and unit checks. Real mission operation should keep live perception/localization inputs fresh and should be tested in `dry_run` before enabling command output.
