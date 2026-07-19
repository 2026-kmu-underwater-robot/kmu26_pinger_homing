#!/usr/bin/env bash
set -euo pipefail

launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch kmu26_pinger_homing pinger_homing_test_tank.launch.py "$@" auto_select_top:=false &
launch_pid=$!

echo "[pinger] Waiting for the five-second frequency scan..."
if ! timeout 30 ros2 topic echo --once /pinger_homing/frequency_candidates; then
  echo "[pinger] No frequency candidates arrived within 30 seconds." >&2
  exit 1
fi

while true; do
  printf '\n\n[Pinger] Candidates are ready. Enter 1-5 or a frequency in Hz, then press Enter:\n' >&2
  read -r selection
  if [[ "${selection}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    ros2 topic pub --once /pinger_homing/manual_selection std_msgs/msg/String \
      "{data: '${selection}'}"
    break
  fi
  echo "Enter 1-5 or a frequency in Hz."
done

wait "${launch_pid}"
