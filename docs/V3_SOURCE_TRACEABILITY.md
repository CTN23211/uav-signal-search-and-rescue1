# V3 source traceability

V3 was reconstructed from the project's available source archives and later recovered files. The purpose of this file is to make the curation boundary explicit.

| V3 file / area | Recovered source basis |
|---|---|
| `ground_station/ros_ws/src/dk_map_builder/` | refactored DK2500 catkin workspace |
| `ground_station/ros_ws/src/dk_search_rviz_panel/` | refactored DK2500 catkin workspace |
| `ground_station/nodes/d435_rgb_tcp_receiver.py` | refactored DK2500 catkin workspace |
| `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py` | later recovered DK2500 receiver source |
| `ground_station/nodes/openvino_yolo_person_detector.py` | later recovered ROS OpenVINO detector source |
| `uav/nodes/jetson_cloud_lora_odom_sender.py` | later recovered Jetson multi-channel sender source |
| `uav/nodes/uav_mission_supervisor.py` | later recovered TCP-enabled mission-supervisor source |
| `uav/nodes/d435_rgb_tcp_sender.py` | deployed Jetson source supplied during V3 reconstruction |
| `uav/ros_ws/src/fox_controller/` | V2 curated archive / original UAV workspace |
| `third_party/fast-drone-250-modifications/` | V2 confirmed modified upstream overlay |

Public-release-only normalization performed during V3 curation:

- removed one deployment-specific absolute waypoint default from `uav_mission_supervisor.py`;
- changed one absolute OpenVINO model default to `/opt/models/...` while keeping it parameterized;
- kept private-LAN IPs / ports as configurable example defaults because they are deployment parameters, not credentials;
- excluded duplicate planner variants, large models, logs, PCDs, ROS build outputs and unmodified third-party trees.
