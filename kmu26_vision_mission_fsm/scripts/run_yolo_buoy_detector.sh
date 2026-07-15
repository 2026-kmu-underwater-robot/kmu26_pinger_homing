#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTOR="${SCRIPT_DIR}/yolo_buoy_detector"

try_python() {
  local candidate="$1"
  shift
  [[ -x "$candidate" ]] || return 1
  "$candidate" -c 'import cv2, rclpy, torch, ultralytics' >/dev/null 2>&1 || return 1
  exec "$candidate" "$DETECTOR" "$@"
}

if [[ -n "${KMU26_VISION_PYTHON:-}" ]]; then
  try_python "$KMU26_VISION_PYTHON" "$@" || {
    echo "configured KMU26_VISION_PYTHON cannot import the vision ROS dependencies: $KMU26_VISION_PYTHON" >&2
    exit 1
  }
fi

for candidate in \
  "$(command -v python3 2>/dev/null || true)" \
  "${COLCON_CURRENT_PREFIX:-}/../../.venvs/uuv_mujoco/bin/python" \
  "${HOME}/.venvs/uuv_mujoco/bin/python" \
  "${HOME}/uuv_sim_current/.venvs/uuv_mujoco/bin/python"
do
  [[ -n "$candidate" ]] || continue
  try_python "$candidate" "$@" || true
done

echo "no Python interpreter can import cv2, rclpy, torch, and ultralytics" >&2
echo "set KMU26_VISION_PYTHON to the prepared interpreter before launch" >&2
exit 1
