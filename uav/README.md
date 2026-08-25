# UAV / Jetson side

## Canonical V3 nodes

- `nodes/jetson_cloud_lora_odom_sender.py` — sends point cloud, LoRa and odometry to DK2500 over separate configurable TCP endpoints.
- `nodes/d435_rgb_tcp_sender.py` — JPEG-compresses D435 RGB and streams it to the ground station.
- `nodes/uav_mission_supervisor.py` — receives high-level routes / commands and publishes sequential `PoseStamped` goals to the local planner.
- `ros_ws/src/fox_controller/` — custom PX4/MAVROS mission-execution package.

FAST-LIO, RealSense ROS, MAVROS and the complete upstream EGO/Fast-Drone-250 tree are external dependencies.
