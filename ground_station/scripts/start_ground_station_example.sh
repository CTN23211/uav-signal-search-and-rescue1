#!/usr/bin/env bash
set -e

# Example only. Source your actual ROS/catkin environments first.
# Adjust ports / model path / launch package to the deployment.

python3 ground_station/nodes/dk2500_cloud_lora_odom_receiver.py &
RX_PID=$!

python3 ground_station/nodes/d435_rgb_tcp_receiver.py &
RGB_PID=$!

cleanup() {
  kill "$RX_PID" "$RGB_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait
