#!/bin/bash
set -e

sudo ip link set can_left down
sudo ip link set can_right down

sudo ip link set can_left name can_tmp
sudo ip link set can_right name can_left
sudo ip link set can_tmp name can_right

sudo ip link set can_left up
sudo ip link set can_right up

echo "Swapped can_left and can_right."
