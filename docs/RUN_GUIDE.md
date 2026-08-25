# Run guide

This guide describes the **integration order**, not a universal one-click flight command. Device names, IPs, ports, frame IDs and safety parameters must be set for the current field setup.

## 1. Ground station first

Start ROS master on the machine chosen for the deployment, then start:

1. `dk2500_cloud_lora_odom_receiver.py`;
2. `d435_rgb_tcp_receiver.py`;
3. the `dk_map_builder` planner launch;
4. the custom RViz panel;
5. `openvino_yolo_person_detector.py` if visual confirmation is required.

Example standalone commands after sourcing ROS:

```bash
python3 ground_station/nodes/dk2500_cloud_lora_odom_receiver.py \
  _cloud_port:=9000 _lora_port:=9001 _odom_port:=9002

python3 ground_station/nodes/d435_rgb_tcp_receiver.py \
  _listen_port:=9200
```

For catkin packages, copy / symlink `ground_station/ros_ws/src/dk_map_builder` and `dk_search_rviz_panel` into a ROS workspace, build, source it, then launch their included launch files.

## 2. UAV localization / camera

Start the deployment-specific components first:

- Livox driver;
- FAST-LIO;
- MAVROS;
- RealSense camera driver.

Confirm that the configured point-cloud, odometry and RGB topics exist before starting transport nodes.

## 3. UAV telemetry

```bash
python3 uav/nodes/jetson_cloud_lora_odom_sender.py \
  _dk2500_ip:=<GROUND_IP> \
  _cloud_port:=<CLOUD_PORT> \
  _lora_port:=<LORA_PORT> \
  _odom_port:=<ODOM_PORT> \
  _lora_serial_port:=<LORA_SERIAL_DEVICE>
```

Start D435 RGB transport separately:

```bash
python3 uav/nodes/d435_rgb_tcp_sender.py \
  _dk_ip:=<GROUND_IP> \
  _dk_port:=<D435_PORT>
```

## 4. Mission supervisor

The supervisor can receive DK2500 route / command JSON over ROS topics or its optional TCP server.

```bash
python3 uav/nodes/uav_mission_supervisor.py \
  _enable_tcp_server:=true \
  _tcp_port:=<MISSION_PORT> \
  _auto_start_file_route:=false
```

If no local waypoint file is provided, the supervisor enters `WAIT_DK_ROUTE` and waits for the ground route.

## 5. EGO / execution layer

Prepare a compatible upstream Fast-Drone-250 / EGO-Planner workspace, then apply / compare the files under:

```text
third_party/fast-drone-250-modifications/overlay/
```

Build and start the local planner and `fox_controller` according to the current aircraft / frame configuration.

## 6. Verify before flight

Before enabling integrated flight, verify at minimum:

- point-cloud topic and frame;
- UAV odometry topic and frame;
- ground-received `/cloud_from_jetson`, `/lora_from_jetson`, `/odom_from_jetson`;
- D435 received image topic;
- RViz planner outputs;
- mission-supervisor acknowledgements / state;
- EGO goal reception and trajectory output;
- RC mode / arm / landing controls;
- all network endpoints are mutually consistent.
