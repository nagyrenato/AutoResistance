#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Ensuring Bluetooth adapter is powered on..."
timeout 5 bluetoothctl power on >/dev/null 2>&1 || true

exec python3 "${SCRIPT_DIR}/domyos_bridge.py" "$@"

