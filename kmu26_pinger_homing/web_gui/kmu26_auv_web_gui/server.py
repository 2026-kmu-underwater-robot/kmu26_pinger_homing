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
    "/vision/buoy/status",
    "/pinger_homing/status",
    "/homing/direction",
    "/pinger_homing/direction_body",
    "/control/pinger/rc_override",
    "/control/rc_override_mux/status",
    "/mission/rviz_markers",
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


def create_app(
    robot_package: str,
    robot_launch: str,
    mission_package: str = "kmu26_vision_mission_fsm",
    mission_launch: str = "mission_fsm_real.launch.py",
    pinger_package: str = "kmu26_pinger_homing",
    pinger_launch: str = "pinger_homing_real.launch.py",
    topic_config: TopicConfig | None = None,
) -> FastAPI:
    app = FastAPI(title="KMU26 AUV Web GUI")
    process_manager = ProcessManager(
        robot_package=robot_package,
        robot_launch=robot_launch,
        mission_package=mission_package,
        mission_launch=mission_launch,
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
        try:
            process_manager.start_stack(body.get("launch_args", {}))
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/stop")
    def stop_stack() -> dict:
        process_manager.stop_stack()
        return _status(process_manager, ros_interface)

    @app.post("/api/mission/start")
    async def start_mission(request: Request) -> dict:
        body = await _json_or_empty(request)
        dry_run = bool(body.get("dry_run", True))
        pinger_homing = bool(body.get("pinger_homing", False))
        launch_args = {
            "use_rviz": "false",
            "use_mission_rviz_visualizer": "true",
            "use_mission_fsm": "true" if not pinger_homing else "false",
            "use_pinger_homing": "true" if pinger_homing else "false",
            "dry_run": "true" if dry_run else "false",
            "require_live_status": "false",
        }
        for key in (
            "pose_topic",
            "pose_type",
            "state_topic",
            "yolo_detection_topic",
            "hydrophone_direction_topic",
            "hydrophone_status_topic",
            "mission_status_json",
            "transport",
            "rate_hz",
            "own_course",
            "course",
        ):
            if key in body and body[key] not in (None, ""):
                launch_args[key] = str(body[key])
        try:
            process_manager.start_mission(launch_args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/mission/stop")
    def stop_mission() -> dict:
        process_manager.stop_mission()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/start")
    async def start_pinger(request: Request) -> dict:
        body = await _json_or_empty(request)
        dry_run = bool(body.get("dry_run", True))
        mission_status = ros_interface.status().get("mission_status", {})
        mission_state = str(mission_status.get("state", "")).upper()
        mission_age = mission_status.get("age")
        mission_is_live = (
            bool(mission_status.get("available", False))
            and isinstance(mission_age, (int, float))
            and mission_age <= 2.0
            and bool(mission_status.get("enabled", True))
            and mission_state not in {"", "COMPLETE", "FAILED", "STOPPED"}
        )
        if mission_is_live:
            raise HTTPException(
                status_code=400,
                detail=(
                    "an active mission status is present; stop the mission before "
                    "starting standalone pinger homing"
                ),
            )
        if not dry_run and not bool(body.get("confirm_live", False)):
            raise HTTPException(
                status_code=400,
                detail="live pinger homing requires confirm_live=true",
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
            "max_runtime_s": str(
                _bounded_float(body, "max_runtime_s", 0.0, 0.0, 3600.0)
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

    @app.post("/api/mission/preflight")
    async def mission_preflight(request: Request) -> dict:
        body = await _json_or_empty(request)
        args = []
        if not bool(body.get("smoke", True)):
            args.append("--no-smoke")
        if not bool(body.get("echo", False)):
            args.append("--no-echo")
        overrides = {
            "--pose-topic": body.get("pose_topic"),
            "--state-topic": body.get("state_topic"),
            "--yolo-topic": body.get("yolo_detection_topic"),
            "--hydrophone-direction-topic": body.get("hydrophone_direction_topic"),
        }
        for flag, value in overrides.items():
            if value not in (None, ""):
                args.extend([flag, str(value)])
        try:
            process_manager.start_preflight(args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--robot-package", default="hit25_auv_ros2")
    parser.add_argument("--robot-launch", default="localization_test.launch.py")
    parser.add_argument("--mission-package", default="kmu26_vision_mission_fsm")
    parser.add_argument("--mission-launch", default="mission_fsm_real.launch.py")
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
        "--yolo-topic",
        default=os.environ.get("KMU26_YOLO_TOPIC", "/vision/buoy/status"),
    )
    parser.add_argument(
        "--pinger-homing-status-topic",
        default=os.environ.get("KMU26_PINGER_HOMING_STATUS_TOPIC", "/pinger_homing/status"),
    )
    parser.add_argument(
        "--hydrophone-direction-topic",
        default=os.environ.get("KMU26_HYDROPHONE_DIRECTION_TOPIC", "/homing/direction"),
    )
    parser.add_argument(
        "--mission-status-json",
        default=os.environ.get("KMU26_MISSION_STATUS_JSON", "/tmp/kmu26_mission_fsm_status.json"),
    )
    args = parser.parse_args()

    topic_config = TopicConfig(
        odom=args.odom_topic,
        dvl_twist=args.dvl_twist_topic,
        depth=args.depth_topic,
        battery=args.battery_topic,
        imu=args.imu_topic,
        joy=args.joy_topic,
        mavros_state=args.mavros_state_topic,
        yolo=args.yolo_topic,
        pinger_homing_status=args.pinger_homing_status_topic,
        hydrophone_direction=args.hydrophone_direction_topic,
        mission_status_json=args.mission_status_json,
    )
    app = create_app(
        robot_package=args.robot_package,
        robot_launch=args.robot_launch,
        mission_package=args.mission_package,
        mission_launch=args.mission_launch,
        pinger_package=args.pinger_package,
        pinger_launch=args.pinger_launch,
        topic_config=topic_config,
    )
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
