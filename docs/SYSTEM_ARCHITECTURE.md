# System architecture

## Functional layers

### 1. UAV sensing / localization

- Livox Mid-360 supplies LiDAR data.
- FAST-LIO supplies the mapped point cloud / pose stream used by the rest of the system.
- MAVROS supplies the flight-controller odometry/state interface.
- LoRa is acquired on the UAV computer through a serial device.
- D435 supplies RGB imagery for close-range confirmation.

### 2. UAV-to-ground transport

`uav/nodes/jetson_cloud_lora_odom_sender.py` is the canonical V3 telemetry sender for:

- point cloud;
- LoRa messages;
- odometry.

`uav/nodes/d435_rgb_tcp_sender.py` separately JPEG-compresses D435 RGB frames and streams them over TCP.

### 3. DK2500 ground station

`ground_station/nodes/dk2500_cloud_lora_odom_receiver.py` republishes received data as ROS topics.

`ground_station/ros_ws/src/dk_map_builder/scripts/uav_3d_search_fusion.py` is the **high-level search-planning core**. It is not an EGO wrapper. It performs the search/fusion logic that chooses valuable search viewpoints and builds stable route sequences.

`ground_station/ros_ws/src/dk_search_rviz_panel/` is the custom RViz operator interface.

### 4. Visual confirmation

`d435_rgb_tcp_receiver.py` decodes the JPEG stream and republishes it as ROS Image.

`openvino_yolo_person_detector.py` runs OpenVINO YOLO person detection and publishes visualization / target events.

### 5. UAV mission supervision

`uav_mission_supervisor.py` is the bridge between high-level route decisions and UAV goal execution. It accepts JSON route/command packets through ROS topics or an optional TCP server and publishes `PoseStamped` goals.

The supervisor deliberately leaves final landing behavior to RC / PX4 / `fox_controller`.

### 6. Local trajectory planning

The project uses an upstream Fast-Drone-250 / EGO-Planner stack with project-specific modified files retained as an overlay. This is the **local planning / collision-avoidance layer**, not the high-level search planner.

### 7. Flight execution

`fox_controller` and MAVROS/PX4 implement the execution layer, including mission setpoint publication and task-state coordination.

## Data-direction summary

```text
UAV sensors -> Jetson -> TCP -> DK2500 -> search fusion -> route
                                                |
                                                +-> RViz operator

DK2500 route -> Mission Supervisor -> EGO -> fox_controller -> MAVROS/PX4

D435 -> JPEG/TCP -> DK2500 -> OpenVINO person detector -> target event
```
