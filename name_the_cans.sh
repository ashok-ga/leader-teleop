#!/usr/bin/env bash
#
# setup_can_left_right.sh
# Individually checks/creates can_left and can_right.

set -euo pipefail

BITRATE=1000000      # 1 Mbit/s

function ensure_interface() {
    FRIENDLY="$1"   # can_left or can_right
    RAW="$2"        # can0 or can1

    # If already exists, bring up and return
    if ip link show "$FRIENDLY" &>/dev/null; then
        sudo ip link set "$FRIENDLY" up
        echo "[INFO] $FRIENDLY already exists – brought UP."
        return
    fi

    # Otherwise, create from raw interface
    if ! ip link show "$RAW" &>/dev/null; then
        echo "[ERROR] $RAW does not exist. Cannot create $FRIENDLY."
        return 1
    fi

    BUSID=$(sudo ethtool -i "$RAW" | awk '/bus-info/ {print $2}')
    echo "[INFO] $RAW bus-id: $BUSID"
    sudo ip link set "$RAW" down
    echo "[INFO] Creating $FRIENDLY from $RAW…"
    sudo bash can_activate.sh "$FRIENDLY" "$BITRATE" "$BUSID"
    sudo ip link set "$FRIENDLY" up
    echo "[SUCCESS] $FRIENDLY is ready."
}

ensure_interface can_left can0
ensure_interface can_right can1

echo "[INFO] Final interface status:"
ip -br link show can_left || true
ip -br link show can_right || true
