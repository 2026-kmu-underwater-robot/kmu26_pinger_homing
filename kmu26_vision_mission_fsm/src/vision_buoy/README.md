# Vision buoy integration

The buoy/stick detector is intentionally kept separate from pinger homing. Its implementation
lives in this ROS 2 package, `kmu26_vision_mission_fsm`; this directory documents the typed
detector/FSM contract while `kmu26_pinger_homing` remains a separate package.

```text
/camera/camera/color/image_raw/compressed
    -> kmu26_vision_mission_fsm
    +-> /vision/buoy_observation -> mission boundary/GUI/RViz
    +-> /vision/buoy_bbox -> dedicated upstream visual FSM
        -> /control/vision/rc_override
    -> rc_override_mux
    -> /mavros/rc/override
```

The typed observation contains the `buoy` or `stick` class, confidence, normalized bbox center
error, bbox size, bearing, and optional estimated range. The FSM prefers `stick` for underwater
rake alignment and accepts `buoy`/`stick` during pinger fine alignment. It uses live odometry and
the configured course boundary to reject targets outside the team's area.

The detector owns image preprocessing, YOLO inference, class selection, and bbox publication. The
dedicated C++ visual FSM, imported from `zetex1001/auv_buoy_vision_control@f782a2e`, owns dive,
visual search, buoy approach, stick alignment, rake insertion/detach/backoff, visual verification,
and its dedicated RC source. The full mission FSM enables it only in `VISION_CONTROL`, monitors its
fresh state, keeps the course/collector policy, and resumes ownership for surface collection and
scoring. It retains the odometry lawnmower search, delegates one detected target at a time, and
takes search ownership back when the upstream FSM returns to `SEARCH` without a target. It rejects
`ASCEND`/`COMPLETE` until the physical collector detach total reaches the full-mission requirement.
The RC mux is the only MAVROS publisher. Neither side may use MuJoCo ground truth as a control input.

`vision_guidance.hpp` and the legacy underwater phases remain as an explicitly disabled fallback
(`use_vision_mission_controller:=false`) for regression comparison. The default launch delegates
to the upstream-derived controller rather than running both control laws at once.
