#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/camera_env.sh"
mkdir -p "$HOME/orbbec_ws/results"
echo $$ > "$HOME/orbbec_ws/results/center_launcher.pid"
exec ros2 run camera_center_test center_point "$@"
