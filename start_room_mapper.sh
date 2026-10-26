#!/usr/bin/env bash
set -euo pipefail

# Start the production JetHexa room-mapping stack safely. This script is
# intended to run on the robot. It never resumes autonomous motion: the mapper
# always starts drive-enabled and AUTO PAUSED.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETHEXA_SETUP="${JETHEXA_SETUP:-$HOME/jethexa/devel/setup.bash}"
LOG_DIR="${ROOM_MAPPER_LOG_DIR:-$HOME/.room_mapper_logs}"
FRESH_MAP=false

usage() {
  echo "Usage: $0 [--fresh-map]"
  echo "  --fresh-map  Reset Hector's current map when starting the mapper."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fresh-map)
      FRESH_MAP=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f "$JETHEXA_SETUP" ]]; then
  echo "JetHexa ROS workspace setup not found: $JETHEXA_SETUP" >&2
  exit 1
fi

if [[ ! -f "$SCRIPT_DIR/hex_map.launch" || ! -f "$SCRIPT_DIR/room_mapper.py" ]]; then
  echo "Run this launcher from the deployed room_mapper project." >&2
  exit 1
fi

set +u
source "$JETHEXA_SETUP"
set -u

if ! rosnode list >/dev/null 2>&1; then
  echo "ROS master is unavailable. Start the JetHexa base system first." >&2
  exit 1
fi

mkdir -p "$LOG_DIR"

node_is_live() {
  rosnode ping -c 1 "$1" >/dev/null 2>&1
}

topic_has_message() {
  local topic="$1"
  local seconds="$2"
  timeout "$seconds" rostopic echo -n 1 "$topic" >/dev/null 2>&1
}

mapper_pids() {
  pgrep -f '(^|/)(python3|python) ([^ ]*/)?room_mapper\.py( |$)' || true
}

mapper_pid_list="$(mapper_pids)"
mapper_count="$(printf '%s\n' "$mapper_pid_list" | sed '/^$/d' | wc -l)"
if [[ "$mapper_count" -gt 1 ]]; then
  echo "Refusing to continue: found $mapper_count room_mapper.py processes." >&2
  printf '%s\n' "$mapper_pid_list" >&2
  exit 1
fi

if [[ "$mapper_count" -eq 1 ]]; then
  echo "Room mapper is already running; no second process was started."
  status="$(curl -fsS --max-time 5 http://127.0.0.1:8000/status || true)"
  if [[ -z "$status" ]]; then
    echo "The existing mapper's web status is unavailable." >&2
    exit 1
  fi
  printf '%s\n' "$status"
  if [[ "$status" != *"AUTO PAUSED"* || "$status" != *"command ('stop',)"* ]]; then
    echo "Existing mapper is not paused and stopped; inspect it directly." >&2
    exit 1
  fi
  exit 0
fi

ros_nodes="$(rosnode list 2>/dev/null || true)"
if printf '%s\n' "$ros_nodes" | grep -Eq '(^|/)(rtabmap|visual_slam)(/|$)'; then
  echo "Refusing to start Hector while the non-production visual SLAM stack is present." >&2
  exit 1
fi

if ! topic_has_message /scan 3; then
  echo "LiDAR has no scan messages; requesting scan startup."
  if rosservice list 2>/dev/null | grep -qx /start_scan; then
    rosservice call /start_scan >/dev/null
  elif ! node_is_live /ydlidar_lidar_g4_publisher; then
    nohup roslaunch jethexa_peripherals lidar.launch publish_tf:=false \
      >"$LOG_DIR/lidar.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    for _ in {1..8}; do
      rosservice list 2>/dev/null | grep -qx /start_scan && break
      sleep 1
    done
    if rosservice list 2>/dev/null | grep -qx /start_scan; then
      rosservice call /start_scan >/dev/null || true
    fi
  fi
fi

if ! topic_has_message /scan 10; then
  echo "LiDAR did not produce /scan messages; inspect $LOG_DIR/lidar.log." >&2
  exit 1
fi
echo "LiDAR is publishing."

hector_started=false
if node_is_live /hector_mapping; then
  echo "Hector is already running; preserving its map unless --fresh-map was requested."
else
  if pgrep -f '(^|/)roslaunch .*hex_map\.launch( |$)' >/dev/null 2>&1; then
    echo "A hex_map.launch process exists without a live Hector node; inspect it before retrying." >&2
    exit 1
  fi
  echo "Starting the production Hector stack with a fresh map."
  nohup roslaunch "$SCRIPT_DIR/hex_map.launch" \
    >"$LOG_DIR/hector.log" 2>&1 < /dev/null &
  disown $! 2>/dev/null || true
  hector_started=true
fi

if ! topic_has_message /slam_out_pose 15; then
  echo "Hector did not produce /slam_out_pose; inspect $LOG_DIR/hector.log." >&2
  exit 1
fi

reset_map=false
if [[ "$hector_started" == true || "$FRESH_MAP" == true ]]; then
  reset_map=true
fi

mapper_pid=""
cleanup_failed_start() {
  local rc=$?
  if [[ $rc -ne 0 && -n "$mapper_pid" ]] && kill -0 "$mapper_pid" 2>/dev/null; then
    echo "Stopping the mapper because startup verification failed." >&2
    kill -INT "$mapper_pid" 2>/dev/null || true
  fi
}
trap cleanup_failed_start EXIT

echo "Starting one drive-enabled mapper in AUTO PAUSED mode."
cd "$SCRIPT_DIR"
nohup /usr/bin/python3 room_mapper.py \
  _drive:=true \
  _start_paused:=true \
  _reset_map:="$reset_map" \
  _detect_objects:=true \
  _use_depth:=true \
  _use_depth_map:=true \
  >"$LOG_DIR/mapper.log" 2>&1 < /dev/null &
mapper_pid=$!
disown "$mapper_pid" 2>/dev/null || true

status=""
for _ in {1..45}; do
  if ! kill -0 "$mapper_pid" 2>/dev/null; then
    break
  fi
  status="$(curl -fsS --max-time 2 http://127.0.0.1:8000/status 2>/dev/null || true)"
  [[ -n "$status" ]] && break
  sleep 1
done

mapper_pid_list="$(mapper_pids)"
mapper_count="$(printf '%s\n' "$mapper_pid_list" | sed '/^$/d' | wc -l)"
if [[ "$mapper_count" -ne 1 ]]; then
  echo "Expected exactly one mapper after startup; found $mapper_count." >&2
  exit 1
fi
if [[ -z "$status" ]]; then
  echo "Mapper web status did not become available; inspect $LOG_DIR/mapper.log." >&2
  exit 1
fi
if [[ "$status" != *"AUTO PAUSED"* || "$status" != *"command ('stop',)"* ]]; then
  printf '%s\n' "$status" >&2
  echo "Mapper did not verify as AUTO PAUSED with command stop." >&2
  exit 1
fi
if [[ "$status" == *"FAULT"* ]]; then
  printf '%s\n' "$status" >&2
  echo "Mapper reported a fault during startup." >&2
  exit 1
fi

trap - EXIT
echo
echo "Room mapper started safely. No movement was commanded."
printf '%s\n' "$status"
echo
echo "Web UI: http://192.168.137.223:8000"
echo "Logs: $LOG_DIR"
