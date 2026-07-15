import argparse
import asyncio
import os
from pathlib import Path

import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi import HTTPException
from fastapi import Request
from fastapi import Response
from fastapi import WebSocket
from fastapi import WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

try:
    from kmu26_pinger_web_gui.ekf_config import read_process_noise_covariance
    from kmu26_pinger_web_gui.ekf_config import write_process_noise_covariance
    from kmu26_pinger_web_gui.process_manager import ProcessManager
    from kmu26_pinger_web_gui.ros_interface import RosInterface
    from kmu26_pinger_web_gui.ros_interface import TopicConfig
except ModuleNotFoundError:
    # Source-tree fallback used by start_pinger_homing_gui.sh before installation.
    from kmu26_auv_web_gui.ekf_config import read_process_noise_covariance
    from kmu26_auv_web_gui.ekf_config import write_process_noise_covariance
    from kmu26_auv_web_gui.process_manager import ProcessManager
    from kmu26_auv_web_gui.ros_interface import RosInterface
    from kmu26_auv_web_gui.ros_interface import TopicConfig


DEFAULT_BAG_TOPICS = [
    "/joy",
    "/battery",
    "/dvl/data",
    "/dvl/twist",
    "/depth/pose",
    "/mavros/imu/data",
    "/mavros/state",
    "/odometry/filtered",
    "/pinger_homing/status",
    "/homing/direction",
    "/pinger_homing/direction_body",
    "/control/pinger/rc_override",
    "/control/rc_override_mux/status",
    "/localization/path",
    "/tf",
    "/tf_static",
]

ALLOWED_DVL_COMMANDS = {
    "calibrate_gyro",
    "get_config",
    "reset_dead_reckoning",
    "set_config",
}

ALLOWED_DVL_PARAMETERS = {
    "",
    "acoustic_enabled",
    "dark_mode_enabled",
    "mountig_rotation_offset",
    "mounting_rotation_offset",
    "range_mode",
    "speed_of_sound",
}

ALLOWED_PINGER_MODES = {"MANUAL", "STABILIZE", "ALT_HOLD", "POSHOLD", "GUIDED"}


def create_app(
    robot_package: str,
    robot_launch: str,
    pinger_package: str = "kmu26_pinger_homing",
    pinger_launch: str = "pinger_homing_real.launch.py",
    topic_config: TopicConfig | None = None,
) -> FastAPI:
    app = FastAPI(title="KMU26 Pinger Homing Web GUI")
    process_manager = ProcessManager(
        robot_package=robot_package,
        robot_launch=robot_launch,
        pinger_package=pinger_package,
        pinger_launch=pinger_launch,
    )
    ros_interface = RosInterface(topic_config=topic_config)
    web_dir_override = os.environ.get("KMU26_WEB_GUI_WEB_DIR")
    if web_dir_override:
        web_dir = Path(web_dir_override)
    else:
        package_share = Path(get_package_share_directory("kmu26_pinger_homing"))
        web_dir = package_share / "web"

    # colcon --symlink-install places web assets behind symlinks outside the
    # installed share directory. Starlette rejects those unless explicitly
    # enabled, which otherwise leaves the GUI unstyled and without JavaScript.
    app.mount(
        "/static",
        StaticFiles(directory=web_dir, follow_symlink=True),
        name="static",
    )

    @app.on_event("startup")
    def on_startup() -> None:
        ros_interface.start()

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        process_manager.stop_all()
        ros_interface.stop()

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(web_dir / "index.html")

    @app.get("/favicon.ico", include_in_schema=False)
    def favicon() -> Response:
        return Response(status_code=204)

    @app.get("/api/status")
    def status() -> dict:
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/start")
    async def start_stack(request: Request) -> dict:
        body = await _json_or_empty(request)
        launch_args = {
            "joy_rc_output_topic": "/control/joystick/rc_override",
            "joy_release_when_idle": "true",
        }
        requested_launch_args = body.get("launch_args", {})
        if not isinstance(requested_launch_args, dict):
            raise HTTPException(status_code=400, detail="launch_args must be an object")
        launch_args.update(requested_launch_args)
        try:
            process_manager.start_stack(launch_args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/stop")
    def stop_stack() -> dict:
        process_manager.stop_stack()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/start")
    async def start_pinger(request: Request) -> dict:
        body = await _json_or_empty(request)
        dry_run = bool(body.get("dry_run", True))
        if not dry_run and not bool(body.get("confirm_live", False)):
            raise HTTPException(
                status_code=400,
                detail="live pinger homing requires confirm_live=true",
            )
        if not dry_run:
            preflight = _pinger_live_preflight(ros_interface.status(), body)
            if not preflight["ok"]:
                failed = "; ".join(
                    check["detail"] for check in preflight["checks"] if not check["ok"]
                )
                raise HTTPException(
                    status_code=400,
                    detail=f"pinger live preflight failed: {failed}",
                )
        amplitude_constant = _bounded_float(
            body, "amplitude_range_constant", 0.0, 0.0, 10.0
        )
        success_range = _bounded_float(body, "success_range_m", 0.0, 0.0, 20.0)
        if success_range > 0.0 and amplitude_constant <= 0.0:
            raise HTTPException(
                status_code=400,
                detail=(
                    "success_range_m requires a calibrated positive "
                    "amplitude_range_constant"
                ),
            )
        topic_config = ros_interface.topic_config
        launch_args = {
            "dry_run": "true" if dry_run else "false",
            "use_audio_capture": (
                "true" if bool(body.get("use_audio_capture", False)) else "false"
            ),
            "use_hydrophone_estimator": (
                "true" if bool(body.get("use_hydrophone_estimator", True)) else "false"
            ),
            "use_rc_mux": "true",
            "audio_device": str(body.get("audio_device", "")).strip(),
            "audio_topic": str(body.get("audio_topic", "/audio")).strip() or "/audio",
            "reference_frequency_hz": str(
                _bounded_float(body, "reference_frequency_hz", 21164.0, 1000.0, 100000.0)
            ),
            "odometry_topic": str(
                body.get("odometry_topic", topic_config.odom)
            ).strip(),
            "depth_topic": str(body.get("depth_topic", topic_config.depth)).strip(),
            "state_topic": str(
                body.get("state_topic", topic_config.mavros_state)
            ).strip(),
            "direction_topic": str(
                body.get("direction_topic", topic_config.hydrophone_direction)
            ).strip(),
            "status_topic": topic_config.pinger_homing_status,
            "rate_hz": str(_bounded_float(body, "rate_hz", 30.0, 1.0, 120.0)),
            "forward_max": str(
                _bounded_float(body, "forward_max", 0.48, 0.05, 0.8)
            ),
            "yaw_gain": str(_bounded_float(body, "yaw_gain", 0.85, 0.1, 2.0)),
            "yaw_command_limit": str(
                _bounded_float(body, "yaw_command_limit", 0.42, 0.05, 0.7)
            ),
            "tank_max_depth_m": str(
                _bounded_float(body, "tank_max_depth_m", 11.0, 0.5, 50.0)
            ),
            "success_range_m": str(success_range),
            "success_hold_s": str(
                _bounded_float(body, "success_hold_s", 0.8, 0.1, 10.0)
            ),
            "arrival_radius_m": str(
                _bounded_float(body, "arrival_radius_m", 1.5, 0.2, 20.0)
            ),
            "arrival_hold_s": str(
                _bounded_float(body, "arrival_hold_s", 1.0, 0.1, 10.0)
            ),
            "max_runtime_s": str(
                _bounded_float(body, "max_runtime_s", 180.0, 5.0, 3600.0)
            ),
            "amplitude_range_constant": str(amplitude_constant),
        }
        try:
            process_manager.start_pinger(launch_args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/stop")
    def stop_pinger() -> dict:
        process_manager.stop_pinger()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/preflight")
    async def pinger_preflight(request: Request) -> dict:
        body = await _json_or_empty(request)
        return _pinger_live_preflight(ros_interface.status(), body)

    @app.post("/api/pinger/arm")
    async def pinger_arm(request: Request) -> dict:
        body = await _json_or_empty(request)
        armed = bool(body.get("armed", False))
        if armed and process_manager.status().get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before changing arm state",
            )
        accepted = ros_interface.set_armed(armed)
        if not accepted:
            raise HTTPException(status_code=503, detail="MAVROS arming service unavailable")
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = True
        return payload

    @app.post("/api/pinger/mode")
    async def pinger_mode(request: Request) -> dict:
        body = await _json_or_empty(request)
        mode = str(body.get("mode", "")).upper()
        if mode not in ALLOWED_PINGER_MODES:
            raise HTTPException(status_code=400, detail=f"unsupported mode: {mode}")
        if process_manager.status().get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before changing vehicle mode",
            )
        accepted = ros_interface.set_mode(mode)
        if not accepted:
            raise HTTPException(status_code=503, detail="MAVROS set-mode service unavailable")
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = True
        return payload

    @app.post("/api/dvl/command")
    async def run_dvl_command(request: Request) -> dict:
        body = await _json_or_empty(request)
        command = str(body.get("command", ""))
        parameter_name = str(body.get("parameter_name", ""))
        parameter_value = str(body.get("parameter_value", ""))
        if command not in ALLOWED_DVL_COMMANDS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL command: {command}")
        if parameter_name not in ALLOWED_DVL_PARAMETERS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL parameter: {parameter_name}")
        if command != "set_config" and (parameter_name or parameter_value):
            raise HTTPException(status_code=400, detail=f"{command} does not accept a parameter")
        if command == "set_config" and not parameter_name:
            raise HTTPException(status_code=400, detail="set_config requires a parameter_name")

        ros_interface.publish_dvl_command(command, parameter_name, parameter_value)
        return _status(process_manager, ros_interface)

    @app.post("/api/dvl/reset_dr")
    def reset_dvl() -> dict:
        ros_interface.reset_dvl_dead_reckoning()
        return _status(process_manager, ros_interface)

    @app.get("/api/ekf/process_noise")
    def get_process_noise() -> dict:
        return read_process_noise_covariance()

    @app.post("/api/ekf/process_noise")
    async def set_process_noise(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            return write_process_noise_covariance(body.get("values", []))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/bag/start")
    async def start_bag(request: Request) -> dict:
        body = await _json_or_empty(request)
        record_all = bool(body.get("record_all", False))
        topics = body.get("topics") or DEFAULT_BAG_TOPICS
        try:
            output_dir = process_manager.start_bag(topics, record_all=record_all)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["bag_output"] = output_dir
        return payload

    @app.post("/api/bag/stop")
    def stop_bag() -> dict:
        process_manager.stop_bag()
        return _status(process_manager, ros_interface)

    @app.get("/api/bag/topics")
    def bag_topics() -> dict:
        try:
            active_topics = process_manager.list_topics()
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        topics = sorted(set(DEFAULT_BAG_TOPICS) | set(active_topics))
        return {
            "default_topics": DEFAULT_BAG_TOPICS,
            "active_topics": active_topics,
            "topics": topics,
        }

    @app.websocket("/ws/status")
    async def status_ws(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(_status(process_manager, ros_interface))
                await asyncio.sleep(0.2)
        except WebSocketDisconnect:
            return

    return app


async def _json_or_empty(request: Request) -> dict:
    try:
        return await request.json()
    except Exception:
        return {}


def _status(process_manager: ProcessManager, ros_interface: RosInterface) -> dict:
    return {
        "process": process_manager.status(),
        "ros": ros_interface.status(),
    }


def _bounded_float(
    body: dict, key: str, default: float, minimum: float, maximum: float
) -> float:
    try:
        value = float(body.get(key, default))
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"{key} must be numeric") from exc
    if not minimum <= value <= maximum:
        raise HTTPException(
            status_code=400,
            detail=f"{key} must be between {minimum} and {maximum}",
        )
    return value


def _pinger_live_preflight(ros_status: dict, body: dict) -> dict:
    topics = ros_status.get("topics", {}) if isinstance(ros_status, dict) else {}
    mavros = ros_status.get("mavros_state", {}) if isinstance(ros_status, dict) else {}
    graph = ros_status.get("graph", {}) if isinstance(ros_status, dict) else {}
    checks: list[dict] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    odom_alive = bool(topics.get("odom", {}).get("alive", False))
    state_alive = bool(topics.get("mavros_state", {}).get("alive", False))
    connected = bool(mavros.get("connected", False))
    armed = bool(mavros.get("armed", False))
    mode = str(mavros.get("mode", "")).upper()
    rc_publishers = int(graph.get("rc_output_publishers", 0) or 0)
    audio_publishers = int(graph.get("audio_publishers", 0) or 0)
    use_capture = bool(body.get("use_audio_capture", False))
    use_estimator = bool(body.get("use_hydrophone_estimator", True))

    add("odometry", odom_alive, "odometry is fresh" if odom_alive else "/odometry/filtered is stale")
    add(
        "mavros",
        state_alive and connected,
        "MAVROS is connected" if state_alive and connected else "/mavros/state is stale or disconnected",
    )
    add("armed", armed, "vehicle is armed" if armed else "vehicle is not armed")
    add(
        "mode",
        mode in ALLOWED_PINGER_MODES,
        f"vehicle mode is {mode}" if mode else "vehicle mode is unavailable",
    )
    add(
        "rc_owner",
        rc_publishers == 0,
        (
            "RC output has no existing publisher"
            if rc_publishers == 0
            else f"/mavros/rc/override already has {rc_publishers} publisher(s)"
        ),
    )
    add("estimator", use_estimator, "hydrophone estimator enabled" if use_estimator else "hydrophone estimator is disabled")
    add(
        "audio",
        use_capture or audio_publishers > 0,
        (
            "audio capture will start"
            if use_capture
            else (
                f"audio source publishers: {audio_publishers}"
                if audio_publishers > 0
                else "/audio has no publisher; enable capture or start the hydrophone input"
            )
        ),
    )
    try:
        max_runtime_s = float(body.get("max_runtime_s", 180.0))
    except (TypeError, ValueError):
        max_runtime_s = 0.0
    try:
        arrival_radius_m = float(body.get("arrival_radius_m", 1.5))
    except (TypeError, ValueError):
        arrival_radius_m = 0.0
    add(
        "runtime_limit",
        max_runtime_s >= 5.0,
        f"runtime limit {max_runtime_s:.1f} s" if max_runtime_s >= 5.0 else "max_runtime_s must be at least 5 s",
    )
    add(
        "arrival_stop",
        arrival_radius_m >= 0.2,
        f"arrival radius {arrival_radius_m:.2f} m" if arrival_radius_m >= 0.2 else "arrival_radius_m must be at least 0.2 m",
    )
    return {"ok": all(check["ok"] for check in checks), "checks": checks}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8878, type=int)
    parser.add_argument("--robot-package", default="hit25_auv_ros2")
    parser.add_argument("--robot-launch", default="localization_test.launch.py")
    parser.add_argument("--pinger-package", default="kmu26_pinger_homing")
    parser.add_argument("--pinger-launch", default="pinger_homing_real.launch.py")
    parser.add_argument("--odom-topic", default=os.environ.get("KMU26_ODOM_TOPIC", "/odometry/filtered"))
    parser.add_argument("--dvl-twist-topic", default=os.environ.get("KMU26_DVL_TWIST_TOPIC", "/dvl/twist"))
    parser.add_argument("--depth-topic", default=os.environ.get("KMU26_DEPTH_TOPIC", "/depth/pose"))
    parser.add_argument("--battery-topic", default=os.environ.get("KMU26_BATTERY_TOPIC", "/battery"))
    parser.add_argument("--imu-topic", default=os.environ.get("KMU26_IMU_TOPIC", "/mavros/imu/data"))
    parser.add_argument("--joy-topic", default=os.environ.get("KMU26_JOY_TOPIC", "/joy"))
    parser.add_argument("--mavros-state-topic", default=os.environ.get("KMU26_MAVROS_STATE_TOPIC", "/mavros/state"))
    parser.add_argument(
        "--pinger-homing-status-topic",
        default=os.environ.get("KMU26_PINGER_HOMING_STATUS_TOPIC", "/pinger_homing/status"),
    )
    parser.add_argument(
        "--hydrophone-direction-topic",
        default=os.environ.get("KMU26_HYDROPHONE_DIRECTION_TOPIC", "/homing/direction"),
    )
    args, _ros_launch_args = parser.parse_known_args()

    topic_config = TopicConfig(
        odom=args.odom_topic,
        dvl_twist=args.dvl_twist_topic,
        depth=args.depth_topic,
        battery=args.battery_topic,
        imu=args.imu_topic,
        joy=args.joy_topic,
        mavros_state=args.mavros_state_topic,
        pinger_homing_status=args.pinger_homing_status_topic,
        hydrophone_direction=args.hydrophone_direction_topic,
    )
    app = create_app(
        robot_package=args.robot_package,
        robot_launch=args.robot_launch,
        pinger_package=args.pinger_package,
        pinger_launch=args.pinger_launch,
        topic_config=topic_config,
    )
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
