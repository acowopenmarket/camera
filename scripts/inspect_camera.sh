#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/camera_env.sh"
out="$HOME/orbbec_ws/results"
mkdir -p "$out"
ros2 node list > "$out/nodes.txt"
ros2 topic list -t > "$out/topics.txt"
ros2 service list -t > "$out/services.txt"
ros2 param dump /camera/camera > "$out/camera_parameters.yaml" || true
timeout 12 ros2 topic echo /camera/color/camera_info --once > "$out/color_camera_info.yaml" || true
timeout 12 ros2 topic echo /camera/depth/camera_info --once > "$out/depth_camera_info.yaml" || true
python3 "$script_dir/capture_tf.py" > "$out/tf_capture_stdout.log"
for topic in /camera/color/image_raw /camera/depth/image_raw; do
  ros2 topic info "$topic" -v
done > "$out/image_topic_details.txt"
cat "$out/nodes.txt" "$out/topics.txt"
