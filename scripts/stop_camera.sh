#!/usr/bin/env bash
set -e
for entry in camera_launch center_launcher; do
  file="$HOME/orbbec_ws/results/$entry.pid"
  if [ -f "$file" ]; then
    read -r pid < "$file"
    if [[ "$pid" =~ ^[0-9]+$ ]] && [ -r "/proc/$pid/cmdline" ]; then
      args=$(tr '\0' ' ' < "/proc/$pid/cmdline")
      case "$args" in
        *'ros2 launch orbbec_camera gemini_330_series.launch.py'*|*'ros2 run camera_center_test center_point'*)
          # ros2 run does not always forward SIGINT to its executable.
          if [[ "$args" == *'ros2 run camera_center_test center_point'* ]]; then
            for child in $(pgrep -P "$pid" || true); do
              if [ -r "/proc/$child/cmdline" ]; then
                child_args=$(tr '\0' ' ' < "/proc/$child/cmdline")
                case "$child_args" in
                  *'/lib/camera_center_test/center_point'*) kill -INT "$child" ;;
                esac
              fi
            done
          fi
          kill -INT "$pid"
          echo "SIGINT sent to camera-only process $pid" ;;
        *) echo "PID $pid no longer matches a camera process; skipped" ;;
      esac
    fi
  fi
done
