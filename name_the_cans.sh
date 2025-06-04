#!/usr/bin/env bash
#
# setup_can_left_right.sh
# -----------------------
# Creates (or just brings up) CAN interfaces called can_left / can_right.
# Uses your existing can_activate.sh helper to do the heavy lifting.
#
set -euo pipefail

BITRATE=1000000      # 1 Mbit/s – change if you need a different speed

# ───────────────────────────────────────────────────────────────────
# 1. Fast-path: if the friendly-named interfaces are already present
# ───────────────────────────────────────────────────────────────────
if ip link show can_left  &>/dev/null && \
   ip link show can_right &>/dev/null; then
    sudo ip link set can_left  up
    sudo ip link set can_right up
    echo "[INFO] can_left and can_right already exist – brought them UP and quit."
    exit 0
fi

# ───────────────────────────────────────────────────────────────────
# 2. Collect bus-IDs for the raw ‘can0’ / ‘can1’ adapters
#    (needed because can_activate.sh expects them)
# ───────────────────────────────────────────────────────────────────
for IF in can0 can1; do
    if ! ip link show "$IF" &>/dev/null; then
        echo "[ERROR] Interface $IF does not exist. Is the CAN adapter plugged in?"
        exit 1
    fi
done

CAN0_BUSID=$(sudo ethtool -i can0 | awk '/bus-info/ {print $2}')
CAN1_BUSID=$(sudo ethtool -i can1 | awk '/bus-info/ {print $2}')

echo "[INFO] Detected bus-IDs:"
echo "       can0 → ${CAN0_BUSID}"
echo "       can1 → ${CAN1_BUSID}"

# ───────────────────────────────────────────────────────────────────
# 3. Bring the raw interfaces DOWN so they can be re-named safely
# ───────────────────────────────────────────────────────────────────
sudo ip link set can0 down
sudo ip link set can1 down

# ───────────────────────────────────────────────────────────────────
# 4. Call can_activate.sh to create the friendly-named interfaces
#    (can_activate.sh <target-ifname> <bitrate> <bus-id>)
# ───────────────────────────────────────────────────────────────────
echo "[INFO] Creating can_left (was can0)…"
sudo bash can_activate.sh can_left  "${BITRATE}" "${CAN0_BUSID}"

echo "[INFO] Creating can_right (was can1)…"
sudo bash can_activate.sh can_right "${BITRATE}" "${CAN1_BUSID}"

# ───────────────────────────────────────────────────────────────────
# 5. Finally bring them UP (paranoia – can_activate usually does this)
# ───────────────────────────────────────────────────────────────────
sudo ip link set can_left  up
sudo ip link set can_right up

echo "[SUCCESS] Interfaces are ready:"
ip -br link show can_left
ip -br link show can_right
