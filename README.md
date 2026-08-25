# UAV Signal-Guided Search and Rescue System

[中文说明 / Chinese Version](README_zh-CN.md)

A ROS1/PX4-based **autonomous UAV search-and-rescue system** for low-altitude distress-signal localization. The system integrates **LoRa wireless-signal sensing**, **Livox Mid-360 LiDAR mapping**, **RealSense D435 visual confirmation**, **DK2500 ground-station search fusion**, **mission supervision**, and **modified EGO-Planner local trajectory planning**.

<p align="center">
  <img src="assets/images/uav_platform.jpg" width="720" alt="UAV platform">
</p>

<p align="center">
  <b>Autonomous UAV platform used in the Zhixun signal-guided search project.</b>
</p>

---

## Project Highlights

- **Signal-guided search:** LoRa RSSI/SNR evidence is fused with spatial information to infer likely distress-source regions.
- **Multi-source perception:** Livox Mid-360 + FAST-LIO for mapping/localization, D435 for RGB visual confirmation.
- **Ground-station intelligence:** DK2500 performs high-level search fusion, route generation, and operator interaction.
- **Autonomous flight execution:** Jetson mission supervisor bridges DK2500 routes to modified EGO-Planner and PX4/MAVROS.
- **Real-flight validated:** The repository includes **indoor** and **outdoor** real-flight demos for direct review.

---

## Real-Flight Demonstrations

<table>
<tr>
<td width="50%" align="center" valign="top">

<a href="assets/demo/demo_indoor_end_to_end.mp4">
  <img src="assets/demo/indoor_demo_thumbnail.jpg" width="100%" alt="Indoor demo thumbnail">
</a>

**Indoor End-to-End Autonomous Search**  
LiDAR mapping, visual verification, and autonomous mission execution.

[▶ Watch Indoor Demo](assets/demo/demo_indoor_end_to_end.mp4)

</td>
<td width="50%" align="center" valign="top">

<a href="assets/demo/demo_outdoor_field_test.mp4">
  <img src="assets/demo/outdoor_demo_thumbnail.jpg" width="100%" alt="Outdoor demo thumbnail">
</a>

**Outdoor Signal-Guided Field Validation**  
Real-flight verification with ground-station visualization and field UAV operation.

[▶ Watch Outdoor Demo](assets/demo/demo_outdoor_field_test.mp4)

</td>
</tr>
</table>

> If GitHub does not preview the MP4 inline in your browser, click the link to open the video file directly.

---

## System Architecture

<p align="center">
  <img src="assets/diagrams/system_architecture.png" width="100%" alt="System architecture diagram">
</p>

### Core runtime logic

1. **UAV sensing and localization**  
   Livox Mid-360 and D435 operate onboard the UAV. FAST-LIO provides pose and point-cloud outputs.
2. **Telemetry forwarding**  
   The Jetson forwards **point cloud + LoRa + odometry** to DK2500, while D435 RGB images are streamed separately over TCP.
3. **Ground-station search fusion**  
   DK2500 fuses terrain / free-space / RF evidence and generates stable high-level search routes.
4. **Mission bridging**  
   `uav_mission_supervisor.py` receives DK2500 route / command messages and publishes planner goals.
5. **Local planning and flight execution**  
   Modified EGO-Planner produces collision-aware local trajectories; `fox_controller` + MAVROS + PX4 execute the task.

---

## Runtime Screenshot

<p align="center">
  <img src="assets/images/search_planning_result.png" width="860" alt="Runtime screenshot">
</p>

<p align="center">
  <b>Ground-station RViz view with point-cloud map, route reasoning, and search-planning result.</b>
</p>

---

## Hardware Platform

### UAV-side components

<table>
<tr>
<td width="33%" align="center"><img src="assets/images/sensor_suite_d435_livox.jpg" width="100%"><br><b>Sensor suite overview</b><br>Livox Mid-360 + RealSense D435</td>
<td width="33%" align="center"><img src="assets/images/livox_mid360_mount.jpg" width="100%"><br><b>Livox Mid-360</b><br>LiDAR module and mount</td>
<td width="33%" align="center"><img src="assets/images/d435_mount.jpg" width="100%"><br><b>RealSense D435</b><br>Front-facing RGB-D camera mount</td>
</tr>
<tr>
<td width="33%" align="center"><img src="assets/images/onboard_computer.jpg" width="100%"><br><b>Onboard computer</b><br>Jetson computing unit</td>
<td width="33%" align="center"><img src="assets/images/flight_controller.jpg" width="100%"><br><b>Flight controller</b><br>PX4 / ArduPilot hardware stack</td>
<td width="33%" align="center"><img src="assets/images/lora_distress_terminal.jpg" width="100%"><br><b>LoRa distress terminal</b><br>Wireless source / rescue beacon</td>
</tr>
</table>

### Ground-station components

<table>
<tr>
<td width="50%" align="center"><img src="assets/images/ground_station.jpg" width="100%"><br><b>Ground station</b><br>Outdoor deployment setup</td>
<td width="50%" align="center"><img src="assets/images/ground_computing_platform.jpg" width="100%"><br><b>Ground computing platform</b><br>DK2500 onboard hardware</td>
</tr>
</table>

---

## Repository Layout

```text
uav-signal-search-and-rescue/
├── assets/
│   ├── demo/
│   ├── diagrams/
│   └── images/
├── config/
├── docs/
├── ground_station/
│   ├── nodes/
│   │   ├── dk2500_cloud_lora_odom_receiver.py
│   │   ├── d435_rgb_tcp_receiver.py
│   │   └── openvino_yolo_person_detector.py
│   └── ros_ws/src/
├── third_party/
│   └── fast-drone-250-modifications/
├── uav/
│   ├── nodes/
│   │   ├── d435_rgb_tcp_sender.py
│   │   ├── jetson_cloud_lora_odom_sender.py
│   │   └── uav_mission_supervisor.py
│   └── ros_ws/src/
├── archive/
├── NOTICE.md
├── README.md
└── README_zh-CN.md
```

---

## Key Source Files

### UAV side
- `uav/nodes/jetson_cloud_lora_odom_sender.py` — forwards cloud / LoRa / odom telemetry to DK2500
- `uav/nodes/d435_rgb_tcp_sender.py` — streams D435 RGB images to DK2500
- `uav/nodes/uav_mission_supervisor.py` — receives route / command messages and dispatches local goals

### Ground station side
- `ground_station/nodes/dk2500_cloud_lora_odom_receiver.py` — receives and republishes telemetry
- `ground_station/nodes/d435_rgb_tcp_receiver.py` — receives D435 RGB image stream
- `ground_station/nodes/openvino_yolo_person_detector.py` — person detection and highlighted bounding boxes
- `ground_station/ros_ws/src/` — search-fusion and RViz-panel related source trees

### Planner / execution related
- `third_party/fast-drone-250-modifications/overlay/` — project-specific EGO / Fast-Drone-250 overlay files
- `docs/EGO_PLANNER_MODIFICATIONS.md` — detailed modification notes

---

## Documentation

- [`docs/RUN_GUIDE.md`](docs/RUN_GUIDE.md) — runtime overview and launch guidance
- [`docs/ROS_INTERFACES.md`](docs/ROS_INTERFACES.md) — ROS topics, interfaces, and integration notes
- [`docs/COMMUNICATION_PROTOCOL.md`](docs/COMMUNICATION_PROTOCOL.md) — telemetry / transport notes
- [`docs/CORE_FILE_LIST.md`](docs/CORE_FILE_LIST.md) — curated V3/V4 core file list
- [`docs/PROJECT_CONTRIBUTIONS.md`](docs/PROJECT_CONTRIBUTIONS.md) — project contribution summary

---

## Important Notes

- **Deployment-specific parameters** such as IP addresses, ports, serial devices, RC mappings, and model paths must be configured according to the real deployment environment.
- This repository is a **curated public-release version** focused on core architecture and representative source files. It intentionally excludes large logs, ROS bags, and bulky generated artifacts.
- The repository includes **project-specific modified / overlay planner files**, but it does **not** claim the full upstream EGO-Planner / Fast-Drone-250 codebase as original work.

---

## Acknowledgement

This repository organizes the core code and demonstration assets of the **Zhixun UAV signal-guided search project** for research, competition showcase, and technical review.
