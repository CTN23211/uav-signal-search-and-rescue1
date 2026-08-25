# V3 core file list by subsystem

## UAV / Jetson

- `uav/nodes/uav_mission_supervisor.py` — mission state machine and DK route/command bridge.
- `uav/nodes/jetson_cloud_lora_odom_sender.py` — canonical cloud / LoRa / odometry sender.
- `uav/nodes/d435_rgb_tcp_sender.py` — D435 RGB JPEG sender.
- `uav/ros_ws/src/fox_controller/` — project-specific PX4/MAVROS mission controller.

## DK2500 / high-level planner

- `ground_station/ros_ws/src/dk_map_builder/scripts/uav_3d_search_fusion.py` — canonical high-level search/fusion/planning core.
- `ground_station/ros_ws/src/dk_map_builder/launch/uav_3d_search_fusion_manual_calib_indicator.launch` — recovered planner launch/configuration.
- `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py` — canonical UAV telemetry receiver.

## RViz

- `ground_station/ros_ws/src/dk_search_rviz_panel/include/dk_search_rviz_panel/search_control_panel.h`
- `ground_station/ros_ws/src/dk_search_rviz_panel/src/search_control_panel.cpp`
- `ground_station/ros_ws/src/dk_search_rviz_panel/launch/search_panel.launch`
- `ground_station/ros_ws/src/dk_search_rviz_panel/rviz/search_panel.rviz`
- `ground_station/ros_ws/src/dk_search_rviz_panel/plugin_description.xml`
- package `CMakeLists.txt` / `package.xml`.

## EGO / local planning

Project-specific modified upstream overlay:

- `third_party/fast-drone-250-modifications/overlay/src/planner/plan_manage/src/ego_replan_fsm.cpp`
- `third_party/fast-drone-250-modifications/overlay/src/planner/plan_manage/src/planner_manager.cpp`
- `advanced_param_exp.xml`
- `single_run_in_exp.launch`
- `single_run_in_vision.launch`
- `single_run_in_fox.launch`
- `default.rviz`

## Communication

- UAV -> DK2500 cloud / LoRa / odometry: `jetson_cloud_lora_odom_sender.py` + `dk2500_cloud_lora_odom_receiver.py`.
- UAV -> DK2500 D435: `d435_rgb_tcp_sender.py` + `d435_rgb_tcp_receiver.py`.
- DK2500 -> UAV route / command: `uav_3d_search_fusion.py` + `uav_mission_supervisor.py`.

## LoRa

- Serial acquisition / parsing is integrated into `uav/nodes/jetson_cloud_lora_odom_sender.py`.
- Network receive and ROS publication are in `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py`.
- RF fusion / source-probability / route influence are in `uav_3d_search_fusion.py`.

## D435

- `uav/nodes/d435_rgb_tcp_sender.py`
- `ground_station/nodes/d435_rgb_tcp_receiver.py`
- `ground_station/nodes/openvino_yolo_person_detector.py`
- output image: `/dk/person_detection/vis`
- target event: `/dk_target_event`

## Non-canonical / archive

- `archive/experimental/raw_livox_to_pcl2_sender_single_socket.py` is retained for traceability only and is not the V3 documented transport path.
