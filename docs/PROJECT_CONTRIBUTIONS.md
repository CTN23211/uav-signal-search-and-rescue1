# Project contribution map

This file separates custom project code from integrated upstream software.

## Project-specific code in V3

### Ground-side search / decision

- `ground_station/ros_ws/src/dk_map_builder/scripts/uav_3d_search_fusion.py`
  - 3D search candidate generation and scoring;
  - obstacle / free-space / terrain evidence;
  - LoRa RF evidence and source-probability logic;
  - stable ordered goals;
  - main / backup route generation;
  - route dispatch to the UAV mission layer.

- `ground_station/ros_ws/src/dk_search_rviz_panel/`
  - custom RViz operator panel and route/task interaction.

### Communication and perception

- `uav/nodes/jetson_cloud_lora_odom_sender.py`
- `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py`
- `uav/nodes/d435_rgb_tcp_sender.py`
- `ground_station/nodes/d435_rgb_tcp_receiver.py`
- `ground_station/nodes/openvino_yolo_person_detector.py`

### Mission / execution integration

- `uav/nodes/uav_mission_supervisor.py`
- `uav/ros_ws/src/fox_controller/`

## Modified upstream code

Files under:

```text
third_party/fast-drone-250-modifications/overlay/
```

are project-specific modifications / additions around the upstream Fast-Drone-250 / EGO-Planner stack. They must be described as **planner adaptation and safety extensions**, not as a from-scratch EGO-Planner implementation.

## External dependencies not vendored

- FAST-LIO / FAST-LIO2
- unmodified Fast-Drone-250 / EGO-Planner tree
- Livox drivers
- RealSense ROS
- PX4 / MAVROS
- OpenVINO model binaries

This separation keeps the public repository focused on the recoverable project contributions while avoiding unnecessary duplication of large upstream codebases.
