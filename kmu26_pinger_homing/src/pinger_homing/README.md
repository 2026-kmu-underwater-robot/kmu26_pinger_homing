# Pinger homing

This directory is the single source of truth for the vehicle-side pinger homing controller.
The hydrophone signal-processing algorithm is not copied into this package. It is consumed from
the team fork of `kmu26_auv_hydrophone` through ROS topics.

## Controllers

- `single_hydrophone_homing_controller.py`: the installed active-range RC controller.
- `single_hydrophone_homing_math.py`: moving-sensor 3D source fit, yaw stabilization, automatic
  vertical probe selection, and tank-depth safety functions used by the active controller.
- `pinger_homing_controller.cpp`: legacy direction/YOLO prototype retained as source history. It is
  intentionally not built or installed by `kmu26_pinger_homing`.

The active controller keeps the tested state order:

```text
WAIT_VEHICLE -> PROBE <-> REPROBE -> ALIGN <-> APPROACH -> CONTACT -> COMPLETE
```

It samples vehicle positions together with phase-derived range changes and coherent-IQ amplitude
ranges. Forward, lateral, and vertical probe legs provide the non-coplanar motion required to fit a
3D stationary source. `tank_max_depth_m` is the only pool-depth input; the controller derives the
vehicle depth limit and chooses the safer vertical probe direction from it.

## Topic boundary

Inputs from the forked hydrophone package:

- `/audio_phase_estimator/delta_range_m` (`std_msgs/Float64`)
- `/audio_phase_estimator/iq_magnitude` (`std_msgs/Float64`)
- `/homing/direction` (`geometry_msgs/Vector3Stamped`, optional fit seed)

Vehicle inputs:

- `/odometry/filtered` (`nav_msgs/Odometry`)
- `/mavros/state` (`mavros_msgs/State`)

Outputs:

- `/control/pinger/rc_override` through `rc_override_mux` in the real launch
- `/pinger_homing/status`
- `/pinger_homing/direction_body`

The controller never reads MuJoCo ground truth. Ground truth is used only by external regression
tests as an oracle.

## Real-vehicle launch

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  dry_run:=true \
  use_audio_capture:=false \
  use_hydrophone_estimator:=true \
  tank_max_depth_m:=11.0
```

`dry_run:=true` computes the same requested command but publishes `CHAN_RELEASE` on every RC
channel. Live mode does not arm the vehicle and activates output only while `/mavros/state` reports
armed. The amplitude range relation `(K / iq_magnitude)^2` requires a measured physical calibration;
the standalone real launch therefore defaults `K=0` and disables metric range completion. The old
`0.325` value is simulator-only. Its dedicated mux refuses to publish when another publisher owns
`/mavros/rc/override`. The patched physical joystick publishes to
`/control/joystick/rc_override`, has higher mux priority, and releases its input while idle.
