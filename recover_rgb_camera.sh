#!/usr/bin/env bash
set -euo pipefail

# Recover only the Orbbec DaBai RGB/UVC USB function. The depth function,
# Hector, room_mapper, and the current map remain running.
RGB_VENDOR="2bc5"
RGB_PRODUCT="0559"
SYS_USB="/sys/bus/usb/devices"

matches=()
for device in "$SYS_USB"/*; do
  [[ -f "$device/idVendor" && -f "$device/idProduct" ]] || continue
  if [[ "$(<"$device/idVendor")" == "$RGB_VENDOR" &&
        "$(<"$device/idProduct")" == "$RGB_PRODUCT" ]]; then
    matches+=("$device")
  fi
done

if [[ ${#matches[@]} -ne 1 ]]; then
  echo "Expected exactly one ${RGB_VENDOR}:${RGB_PRODUCT} RGB device; found ${#matches[@]}" >&2
  exit 1
fi

rgb_device="${matches[0]}"
echo "Recovering RGB USB function ${rgb_device##*/} (${RGB_VENDOR}:${RGB_PRODUCT})"

pid="$(rosnode info /camera/uvc_color 2>/dev/null | awk '/^Pid:/{print $2; exit}' || true)"
rosnode kill /camera/uvc_color >/dev/null 2>&1 || true
if [[ -n "$pid" ]]; then
  for _ in {1..20}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill -9 "$pid" 2>/dev/null || true
  fi
fi

sudo sh -c "echo 0 > '$rgb_device/authorized'; sleep 2; echo 1 > '$rgb_device/authorized'"
sleep 2

log="/tmp/room_mapper_uvc_recovery.log"
ROS_NAMESPACE=/camera nohup rosrun astra_camera camera_node \
  image_raw:=rgb/image_raw __name:=uvc_color >"$log" 2>&1 &
camera_pid=$!
disown "$camera_pid" 2>/dev/null || true

echo "Started /camera/uvc_color as PID $camera_pid; checking RGB frames..."
if timeout 10 rostopic hz -w 8 /camera/rgb/image_raw 2>&1 | grep -q "average rate"; then
  echo "RGB recovery succeeded"
else
  echo "RGB recovery failed; inspect $log and dmesg" >&2
  exit 1
fi
