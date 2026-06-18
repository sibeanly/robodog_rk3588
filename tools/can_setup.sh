#!/bin/bash
# SPDX-License-Identifier: GPL-3.0
# Bring up native mttcan SocketCAN interfaces on Jetson Thor for mevius2 quadruped.
# mevius2 uses can0 (BL+BR legs) and can1 (FL+FR legs), 1 Mbps, CAN 2.0 classic.
# These are native nvidia,tegra264-mttcan controllers (no USB-CAN adapter needed).

set -e

IFACES=("can0" "can1")
BITRATE=1000000

for iface in "${IFACES[@]}"; do
    echo "[$iface] bringing up @ ${BITRATE}bps"
    sudo ip link set "$iface" down 2>/dev/null || true
    sudo ip link set "$iface" type can bitrate "$BITRATE"
    sudo ip link set "$iface" up
    if ip link show "$iface" | grep -q "state UP"; then
        echo "[$iface] UP ✓"
    else
        echo "[$iface] FAILED to come up" >&2
        exit 1
    fi
done
echo "CAN interfaces ready: ${IFACES[*]}"
