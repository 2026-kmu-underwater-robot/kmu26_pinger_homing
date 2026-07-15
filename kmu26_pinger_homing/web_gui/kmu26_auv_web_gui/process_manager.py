import os
import signal
import subprocess
import sys
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Iterable


def subprocess_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
    return env


class ManagedProcess:
    def __init__(self, name: str, cmd: list[str], log_buffer: deque[str]):
        self.name = name
        self.cmd = cmd
        self.log_buffer = log_buffer
        self.process: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError(f"{self.name} is already running")

        self._log(f"$ {' '.join(self.cmd)}")
        self.process = subprocess.Popen(
            self.cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=subprocess_env(),
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self._reader_thread.start()

    def stop(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return

        self._log(f"[{self.name}] stopping")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._log(f"[{self.name}] force killing")
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait(timeout=2.0)

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def _read_output(self) -> None:
        assert self.process is not None
        if self.process.stdout is None:
            return
        for line in self.process.stdout:
            self._log(f"[{self.name}] {line.rstrip()}")
        return_code = self.process.wait()
        self._log(f"[{self.name}] exited with code {return_code}")

    def _log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        entry = f"{timestamp} {message}"
        self.log_buffer.append(entry)
        print(entry, file=sys.stdout, flush=True)

class ProcessManager:
    def __init__(
        self,
        robot_package: str = "hit25_auv_ros2",
        robot_launch: str = "localization_test.launch.py",
        mission_package: str = "kmu26_vision_mission_fsm",
        mission_launch: str = "mission_fsm_real.launch.py",
        pinger_package: str = "kmu26_pinger_homing",
        pinger_launch: str = "pinger_homing_real.launch.py",
    ):
        self.robot_package = robot_package
        self.robot_launch = robot_launch
        self.mission_package = mission_package
        self.mission_launch = mission_launch
        self.pinger_package = pinger_package
        self.pinger_launch = pinger_launch
        self.logs: deque[str] = deque(maxlen=500)
        self._stack: ManagedProcess | None = None
        self._mission: ManagedProcess | None = None
        self._pinger: ManagedProcess | None = None
        self._preflight: ManagedProcess | None = None
        self._bag: ManagedProcess | None = None
        self._bag_output: str = ""

    def start_stack(self, launch_args: dict[str, str] | None = None) -> None:
        if self._stack and self._stack.is_running:
            raise RuntimeError("robot stack is already running")
        args = launch_args or {}
        cmd = ["ros2", "launch", self.robot_package, self.robot_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")

        self._stack = ManagedProcess("localization_test", cmd, self.logs)
        self._stack.start()

    def stop_stack(self) -> None:
        if self._stack:
            self._stack.stop()

    def start_mission(self, launch_args: dict[str, str] | None = None) -> None:
        if self._mission and self._mission.is_running:
            raise RuntimeError("mission FSM is already running")
        if self._pinger and self._pinger.is_running:
            raise RuntimeError("stop standalone pinger homing before starting the mission FSM")
        args = {
            "use_rviz": "false",
            "use_mission_rviz_visualizer": "true",
            "use_mission_fsm": "true",
            "dry_run": "true",
            "require_live_status": "false",
        }
        args.update(launch_args or {})
        cmd = ["ros2", "launch", self.mission_package, self.mission_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")

        self._mission = ManagedProcess("mission_fsm", cmd, self.logs)
        self._mission.start()

    def stop_mission(self) -> None:
        if self._mission:
            self._mission.stop()

    def start_pinger(self, launch_args: dict[str, str] | None = None) -> None:
        if self._pinger and self._pinger.is_running:
            raise RuntimeError("pinger homing is already running")
        if self._mission and self._mission.is_running:
            raise RuntimeError("stop the mission FSM before starting standalone pinger homing")
        args = {
            "dry_run": "true",
            "use_audio_capture": "false",
            "use_hydrophone_estimator": "true",
            "use_rc_mux": "true",
        }
        args.update(launch_args or {})
        cmd = ["ros2", "launch", self.pinger_package, self.pinger_launch]
        for key, value in args.items():
            if value != "":
                cmd.append(f"{key}:={value}")
        self._pinger = ManagedProcess("pinger_homing", cmd, self.logs)
        self._pinger.start()

    def stop_pinger(self) -> None:
        if self._pinger:
            self._pinger.stop()

    def start_preflight(self, args: Iterable[str] | None = None) -> None:
        if self._preflight and self._preflight.is_running:
            raise RuntimeError("preflight is already running")
        cmd = ["ros2", "run", self.mission_package, "real_vehicle_preflight.sh"]
        cmd.extend(args or [])
        self._preflight = ManagedProcess("preflight", cmd, self.logs)
        self._preflight.start()

    def stop_preflight(self) -> None:
        if self._preflight:
            self._preflight.stop()

    def start_bag(
        self,
        topics: Iterable[str] | None = None,
        output_root: str = "~/auv_localization_bags",
        record_all: bool = False,
    ) -> str:
        if self._bag and self._bag.is_running:
            raise RuntimeError("bag recording is already running")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(output_root).expanduser() / f"localization_{timestamp}"
        output_dir.parent.mkdir(parents=True, exist_ok=True)
        topic_list = [topic for topic in (topics or []) if topic]
        if record_all:
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), "-a"]
        else:
            if not topic_list:
                raise RuntimeError("at least one topic must be selected")
            cmd = ["ros2", "bag", "record", "-o", str(output_dir), *topic_list]
        self._bag = ManagedProcess("bag", cmd, self.logs)
        self._bag_output = str(output_dir)
        self._bag.start()
        return str(output_dir)

    def list_topics(self) -> list[str]:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=subprocess_env(),
            text=True,
            timeout=3.0,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or "failed to list ROS topics")
        return sorted(line.strip() for line in result.stdout.splitlines() if line.strip())

    def stop_bag(self) -> None:
        if self._bag:
            self._bag.stop()

    def status(self) -> dict:
        return {
            "stack_running": bool(self._stack and self._stack.is_running),
            "mission_running": bool(self._mission and self._mission.is_running),
            "pinger_running": bool(self._pinger and self._pinger.is_running),
            "preflight_running": bool(self._preflight and self._preflight.is_running),
            "bag_running": bool(self._bag and self._bag.is_running),
            "bag_output": self._bag_output,
            "logs": list(self.logs)[-80:],
        }

    def stop_all(self) -> None:
        self.stop_bag()
        self.stop_preflight()
        self.stop_pinger()
        self.stop_mission()
        self.stop_stack()
