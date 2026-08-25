# V3 release notes

## Major change

V3 changes the repository from a UAV-heavy code snapshot into a recovered **end-to-end UAV–ground search system**.

## Restored / added to the canonical tree

- complete DK2500 high-level search planner package;
- complete custom RViz search panel package;
- Jetson cloud / LoRa / odometry transport sender;
- DK2500 cloud / LoRa / odometry transport receiver;
- D435 RGB TCP sender and receiver;
- ROS OpenVINO YOLO person detector;
- TCP-enabled UAV mission supervisor;
- custom `fox_controller`;
- confirmed EGO / Fast-Drone-250 modification overlay.

## Architecture correction

The documentation now explicitly distinguishes:

- **DK2500 high-level search planning**;
- **EGO local trajectory planning / avoidance**;
- **Mission Supervisor route-to-goal bridging**;
- **PX4 / MAVROS execution**.

## Repository cleanup

The V3 main tree excludes duplicate historical planner variants and non-core V2 competition packages. Their removal from the V3 snapshot does not erase prior Git history.

## Deployment configuration

IP addresses, ports, serial devices and model paths are documented as field parameters rather than fixed architecture constants.
