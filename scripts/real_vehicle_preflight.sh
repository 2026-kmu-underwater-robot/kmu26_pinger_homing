#!/usr/bin/env bash
# Real-vehicle preflight for kmu26_mission_fsm.
# This script sources ROS/workspace setup only inside this process.

set -u

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0
DO_BUILD=0
DO_SMOKE=1
DO_ECHO=1
ECHO_TIMEOUT_S=2
POSE_TOPIC="/odometry/filtered"
POSE_TYPE="nav_msgs/msg/Odometry"
STATE_TOPIC="/mavros/state"
STATE_TYPE="mavros_msgs/msg/State"
YOLO_TOPIC="/uuv_mujoco/yolo_buoy_detections"
YOLO_TYPE="std_msgs/msg/String"
HYDROPHONE_DIRECTION_TOPIC="/mujoco/hydrophone/direction"
HYDROPHONE_DIRECTION_TYPE="geometry_msgs/msg/Vector3Stamped"

usage() {
  cat <<'EOF'
Usage: real_vehicle_preflight.sh [--build] [--no-smoke] [--no-echo]
                                 [--pose-topic TOPIC] [--state-topic TOPIC]
                                 [--yolo-topic TOPIC]
                                 [--hydrophone-direction-topic TOPIC]

Checks that the installed kmu26_mission_fsm package matches the pulled source
and that real-vehicle launch defaults/topics are sane. Use --build after git
pull to apply the source into the workspace install before checking.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1 ;;
    --no-smoke) DO_SMOKE=0 ;;
    --no-echo) DO_ECHO=0 ;;
    --pose-topic)
      [[ $# -ge 2 ]] || { echo "[FAIL] --pose-topic requires a value"; exit 2; }
      POSE_TOPIC="$2"
      shift
      ;;
    --state-topic)
      [[ $# -ge 2 ]] || { echo "[FAIL] --state-topic requires a value"; exit 2; }
      STATE_TOPIC="$2"
      shift
      ;;
    --yolo-topic)
      [[ $# -ge 2 ]] || { echo "[FAIL] --yolo-topic requires a value"; exit 2; }
      YOLO_TOPIC="$2"
      shift
      ;;
    --hydrophone-direction-topic)
      [[ $# -ge 2 ]] || { echo "[FAIL] --hydrophone-direction-topic requires a value"; exit 2; }
      HYDROPHONE_DIRECTION_TOPIC="$2"
      shift
      ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "[FAIL] unknown argument: $1"
      usage
      exit 2
      ;;
  esac
  shift
done

pass() { echo "[PASS] $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
warn() { echo "[WARN] $*"; WARN_COUNT=$((WARN_COUNT + 1)); }
fail() { echo "[FAIL] $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
info() { echo "[INFO] $*"; }

script_path="${BASH_SOURCE[0]}"
while [ -L "$script_path" ]; do
  script_dir="$(cd "$(dirname "$script_path")" && pwd)"
  script_path="$(readlink "$script_path")"
  [[ "$script_path" != /* ]] && script_path="$script_dir/$script_path"
done
SCRIPT_DIR="$(cd "$(dirname "$script_path")" && pwd)"

PKG_SRC=""
WS_DIR=""
if [[ -f "$SCRIPT_DIR/../package.xml" ]]; then
  PKG_SRC="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [[ "$SCRIPT_DIR" == */install/kmu26_mission_fsm/lib/kmu26_mission_fsm ]]; then
  prefix="${SCRIPT_DIR%/lib/kmu26_mission_fsm}"
  WS_DIR="${prefix%/install/kmu26_mission_fsm}"
  if [[ -f "$WS_DIR/src/kmu26_mission_fsm/package.xml" ]]; then
    PKG_SRC="$WS_DIR/src/kmu26_mission_fsm"
  fi
fi

if [[ -n "$PKG_SRC" && "$PKG_SRC" == */src/kmu26_mission_fsm ]]; then
  WS_DIR="${PKG_SRC%/src/kmu26_mission_fsm}"
elif [[ -z "$WS_DIR" && -n "$PKG_SRC" ]]; then
  candidate="$(cd "$PKG_SRC/../.." 2>/dev/null && pwd || true)"
  [[ -f "$candidate/install/setup.bash" || -d "$candidate/src" ]] && WS_DIR="$candidate"
  [[ -z "$WS_DIR" && -f "$PKG_SRC/install/setup.bash" ]] && WS_DIR="$PKG_SRC"
fi

info "package source: ${PKG_SRC:-unknown}"
info "workspace: ${WS_DIR:-unknown}"

ros_distro="${ROS_DISTRO:-humble}"
if [[ -f "/opt/ros/$ros_distro/setup.bash" ]]; then
  set +u
  source "/opt/ros/$ros_distro/setup.bash"
  set -u
  pass "sourced /opt/ros/$ros_distro/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  pass "sourced /opt/ros/humble/setup.bash"
else
  fail "ROS 2 setup.bash not found under /opt/ros"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 command not available"
fi

if [[ "$DO_BUILD" -eq 1 ]]; then
  if [[ -z "$WS_DIR" || ! -d "$WS_DIR" ]]; then
    fail "--build requested but workspace root was not detected"
  elif ! command -v colcon >/dev/null 2>&1; then
    fail "--build requested but colcon is not available"
  else
    info "building kmu26_mission_fsm in $WS_DIR"
    if (cd "$WS_DIR" && colcon build --packages-select kmu26_mission_fsm); then
      pass "colcon build kmu26_mission_fsm"
    else
      fail "colcon build kmu26_mission_fsm"
    fi
  fi
fi

if [[ -n "$WS_DIR" && -f "$WS_DIR/install/setup.bash" ]]; then
  set +u
  source "$WS_DIR/install/setup.bash"
  set -u
  pass "sourced workspace install/setup.bash"
else
  warn "workspace install/setup.bash not found; package may not be visible until built"
fi

PKG_PREFIX=""
if command -v ros2 >/dev/null 2>&1; then
  PKG_PREFIX="$(ros2 pkg prefix kmu26_mission_fsm 2>/dev/null || true)"
fi
if [[ -n "$PKG_PREFIX" ]]; then
  pass "package visible: $PKG_PREFIX"
else
  fail "package not visible to ros2; run this from the workspace or use --build"
fi

if [[ -n "$PKG_PREFIX" && -n "$PKG_SRC" ]]; then
  installed_launch="$PKG_PREFIX/share/kmu26_mission_fsm/launch/mission_fsm_real.launch.py"
  source_launch="$PKG_SRC/launch/mission_fsm_real.launch.py"
  if [[ -f "$installed_launch" && -f "$source_launch" ]]; then
    if cmp -s "$source_launch" "$installed_launch"; then
      pass "installed launch matches pulled source"
    else
      fail "installed launch is stale; run: colcon build --packages-select kmu26_mission_fsm"
    fi
  else
    warn "could not compare source/install launch files"
  fi
fi

SHOW_ARGS=""
if [[ -n "$PKG_PREFIX" ]]; then
  if SHOW_ARGS="$(ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py --show-args 2>&1)"; then
    pass "launch file parses"
  else
    fail "launch file failed to parse"
    echo "$SHOW_ARGS"
  fi
fi

check_launch_default() {
  local name="$1"
  local expected="$2"
  if grep -A4 "'$name':" <<<"$SHOW_ARGS" | grep -Fq "(default: '$expected')"; then
    pass "launch default $name=$expected"
  else
    fail "launch default $name is not $expected"
  fi
}

if [[ -n "$SHOW_ARGS" ]]; then
  check_launch_default "use_rviz" "false"
  check_launch_default "pose_topic" "/odometry/filtered"
  check_launch_default "pose_type" "odometry"
  check_launch_default "marker_frame" "odom"
  check_launch_default "require_live_status" "false"
fi

if [[ -n "${DISPLAY:-}" ]]; then
  if [[ "$DISPLAY" == localhost:* ]]; then
    warn "DISPLAY=$DISPLAY looks like SSH X forwarding; keep use_rviz:=false on the NUC/container"
  else
    pass "DISPLAY is set: $DISPLAY"
  fi
else
  pass "DISPLAY is unset; headless launch default is expected"
fi

TOPICS="$(timeout 4s ros2 topic list -t 2>/dev/null || true)"
if [[ -n "$TOPICS" ]]; then
  pass "ROS graph is reachable"
else
  warn "no ROS topics visible; check ROS_DOMAIN_ID, RMW, and robot bringup"
fi

check_topic() {
  local topic="$1"
  local type="$2"
  local required="$3"
  if grep -Fq "$topic [$type]" <<<"$TOPICS"; then
    pass "$topic type=$type"
    return 0
  fi
  if [[ "$required" == "required" ]]; then
    fail "$topic type=$type not visible"
  else
    warn "$topic type=$type not visible"
  fi
  return 1
}

show_topic_candidates() {
  local label="$1"
  local pattern="$2"
  local matches
  matches="$(grep -Ei "$pattern" <<<"$TOPICS" | sed -n '1,12p' || true)"
  if [[ -n "$matches" ]]; then
    warn "$label candidates visible:"
    sed 's/^/[INFO]   /' <<<"$matches"
  else
    warn "no $label-like topics visible"
  fi
}

topic_echo_once() {
  local topic="$1"
  local label="$2"
  if [[ "$DO_ECHO" -eq 0 ]]; then
    return
  fi
  if timeout "${ECHO_TIMEOUT_S}s" ros2 topic echo --once "$topic" >/tmp/kmu26_mission_fsm_preflight_echo.txt 2>&1; then
    pass "$label publishes messages"
  else
    warn "$label did not publish within ${ECHO_TIMEOUT_S}s"
  fi
}

if check_topic "$POSE_TOPIC" "$POSE_TYPE" "required"; then
  topic_echo_once "$POSE_TOPIC" "$POSE_TOPIC"
else
  show_topic_candidates "odometry" "odom|odometry|ekf|filter|pose"
fi
if check_topic "$STATE_TOPIC" "$STATE_TYPE" "required"; then
  topic_echo_once "$STATE_TOPIC" "$STATE_TOPIC"
else
  show_topic_candidates "mavros/state" "mavros|state"
fi
check_topic "$YOLO_TOPIC" "$YOLO_TYPE" "optional" || show_topic_candidates "YOLO detection" "yolo|detect|vision|buoy"
check_topic "$HYDROPHONE_DIRECTION_TOPIC" "$HYDROPHONE_DIRECTION_TYPE" "optional" ||
  show_topic_candidates "hydrophone direction" "hydro|pinger|phase|bearing|direction"
if ! grep -Fq "/mujoco/course_buoys/status [std_msgs/msg/String]" <<<"$TOPICS"; then
  pass "MuJoCo buoy status is absent; OK because require_live_status default is false"
fi

if [[ "$DO_SMOKE" -eq 1 && -n "$PKG_PREFIX" ]]; then
  smoke_log="$(mktemp /tmp/kmu26_mission_fsm_smoke.XXXXXX.log)"
  info "running 4s headless launch smoke; log=$smoke_log"
  timeout 4s ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py use_rviz:=false \
    >"$smoke_log" 2>&1
  smoke_code=$?
  if [[ "$smoke_code" -eq 124 ]]; then
    if grep -Eq "Traceback|\\[ERROR\\]|process has died|could not connect to display|qt.qpa.xcb" "$smoke_log"; then
      fail "headless launch smoke reported errors"
      tail -n 40 "$smoke_log"
    else
      pass "headless launch smoke"
    fi
  elif [[ "$smoke_code" -eq 0 ]]; then
    pass "headless launch smoke exited cleanly"
  else
    fail "headless launch smoke failed with exit code $smoke_code"
    tail -n 40 "$smoke_log"
  fi
fi

echo
echo "Summary: PASS=$PASS_COUNT WARN=$WARN_COUNT FAIL=$FAIL_COUNT"
if [[ "$FAIL_COUNT" -gt 0 ]]; then
  exit 1
fi
exit 0
