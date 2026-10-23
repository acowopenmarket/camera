#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/handeye_env.sh"
cd "$camera_project_root"
exec python3 -m handeye_calibration.session "$@"
