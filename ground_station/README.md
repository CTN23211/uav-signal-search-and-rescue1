# DK2500 ground station

## Canonical V3 components

- `nodes/dk2500_cloud_lora_odom_receiver.py` — receives UAV telemetry and republishes ROS topics.
- `nodes/d435_rgb_tcp_receiver.py` — receives D435 JPEG frames and republishes ROS Image.
- `nodes/openvino_yolo_person_detector.py` — OpenVINO YOLO person detector and target-event publisher.
- `ros_ws/src/dk_map_builder/` — high-level 3D search / RF fusion / route planner.
- `ros_ws/src/dk_search_rviz_panel/` — custom RViz mission panel.

The ground planner is the high-level search brain. EGO remains the UAV-side local trajectory planner.
