#!/bin/bash
set -e

BITRATE=1000000

# If both exist, just bring up and exit
if ip link show can_left &>/dev/null && ip link show can_right &>/dev/null; then
  sudo ip link set can_left up
  sudo ip link set can_right up
  echo "can_left and can_right already exist. Brought them up."
  exit 0
fi

# Otherwise, activate can0/can1 and rename as before
for IF in can0 can1; do
  BUSID=$(sudo ethtool -i "$IF" | grep bus | awk '{print $2}')
  echo "Interface $IF has BUSID $BUSID"
  sudo bash can_activate.sh "$IF" $BITRATE "$BUSID"
done

# Now determine which is left/right by busid
CAN0_BUSID=$(sudo ethtool -i can0 | grep bus | awk '{print $2}')
CAN1_BUSID=$(sudo ethtool -i can1 | grep bus | awk '{print $2}')

if [ "$CAN0_BUSID" = "$LEFT_BUS" ]; then
  LEFT_IF=can0
  RIGHT_IF=can1
elif [ "$CAN1_BUSID" = "$LEFT_BUS" ]; then
  LEFT_IF=can1
  RIGHT_IF=can0
else
  echo "ERROR: Neither can0 nor can1 matches LEFT_BUS."
  exit 1
fi

sudo ip link set "$LEFT_IF" down
sudo ip link set "$RIGHT_IF" down

sudo ip link set "$LEFT_IF" name can_left
sudo ip link set "$RIGHT_IF" name can_right

sudo ip link set can_left up
sudo ip link set can_right up

echo "Activated and renamed CAN interfaces:"
echo "$LEFT_IF ($LEFT_BUS) → can_left"
echo "$RIGHT_IF ($RIGHT_BUS) → can_right"
