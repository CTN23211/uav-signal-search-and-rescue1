# ROS interfaces

This table documents the main default topics visible in the recovered V3 nodes. All are ROS parameters and can be remapped.

| Layer | Default topic | Type / role |
|---|---|---|
| FAST-LIO -> UAV transport | `/cloud_registered` | `sensor_msgs/PointCloud2` |
| MAVROS -> UAV transport | `/mavros/local_position/odom` | `nav_msgs/Odometry` |
| DK receiver -> planner | `/cloud_from_jetson` | `sensor_msgs/PointCloud2` |
| DK receiver -> planner | `/lora_from_jetson` | `std_msgs/String` JSON |
| DK receiver -> planner | `/odom_from_jetson` | `nav_msgs/Odometry` |
| D435 camera -> sender | `/camera/color/image_raw` | `sensor_msgs/Image` |
| D435 receiver -> detector | `/d435/rgb_from_jetson` | `sensor_msgs/Image` |
| Detector visualization | `/dk/person_detection/vis` | `sensor_msgs/Image` |
| Detector target event | `/dk_target_event` | `std_msgs/String` JSON |
| Detector boolean | `/dk/person_detected` | `std_msgs/Bool` |
| Ground route trigger | `/send_route_to_uav` | `std_msgs/Bool` |
| Ground mission command | `/dk_tcp_mission_cmd` | `std_msgs/String` JSON |
| Mission supervisor goal | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` |
| Mission supervisor status | `/uav_exec_status` | `std_msgs/String` JSON |
| Route acknowledgement | `/uav_route_ack` | `std_msgs/String` JSON |
| Current UAV goal | `/uav_current_goal` | `std_msgs/String` JSON |
| External goal-reached input | `/uav_goal_reached` | `std_msgs/Bool` |

The ground planner publishes additional map, RF, score, route and marker topics. Their full parameter set is defined directly in `uav_3d_search_fusion.py` and its launch file.
