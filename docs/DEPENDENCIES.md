# Dependencies

## Core platform

- Ubuntu 20.04
- ROS Noetic
- Python 3
- C++14-capable toolchain

## UAV / flight

- PX4
- MAVROS (`mavros_msgs`)
- `geometry_msgs`, `nav_msgs`, `std_msgs`, `sensor_msgs`

## LiDAR / localization

- Livox ROS driver compatible with Mid-360
- FAST-LIO / FAST-LIO2
- PCL / ROS PointCloud2 stack as required by upstream packages

## Ground search / visualization

- NumPy
- RViz
- Qt5
- `pluginlib`
- `visualization_msgs`

## D435 / vision

- Intel RealSense ROS driver
- OpenCV
- `cv_bridge`
- OpenVINO Runtime
- compatible YOLO person-detection OpenVINO model

## LoRa

- pyserial
- a serial source producing the expected `LORA_RX,...` line format, or adapt `parse_lora_line()` to your receiver firmware.

## Planner

- compatible upstream Fast-Drone-250 / EGO-Planner source tree
- project overlay from `third_party/fast-drone-250-modifications/overlay/`

The exact upstream base commit used in the original workspace was not preserved.
