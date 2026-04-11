#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR"

set +u
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
set -u

exec python3 scripts/bridge方案/camera_bridge_receiver.py "$@"
