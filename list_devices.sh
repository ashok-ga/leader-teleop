#!/bin/bash

echo "devices:"
echo "  cameras:"

for dev in /dev/video*; do
  formats=$(v4l2-ctl --device=$dev --list-formats-ext 2>/dev/null)

  if echo "$formats" | grep -q -E "MJPG|YUYV|RGB3"; then
    model=$(udevadm info --query=property --name=$dev 2>/dev/null | grep ID_MODEL= | cut -d= -f2)
    serial=$(udevadm info --query=property --name=$dev 2>/dev/null | grep ID_SERIAL= | cut -d= -f2)
    path_link=$(readlink -f "$dev")

    echo "    - device: \"$path_link\""
    [ -n "$model" ] && echo "      model: \"$model\""
    [ -n "$serial" ] && echo "      serial: \"$serial\""
  fi
done



echo "Listing connected ttyUSB devices..."
echo ""

for dev in /dev/ttyUSB*; do
    [ -e "$dev" ] || continue

    echo "Device: $dev"
    udevadm info --query=property --name="$dev" | grep -E 'DEVNAME|ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL|ID_VENDOR'
    echo ""
done