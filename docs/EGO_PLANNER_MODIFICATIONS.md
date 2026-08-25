# EGO-Planner / Fast-Drone-250 modifications

This repository does **not** claim ownership of EGO-Planner or Fast-Drone-250.

The original workspace used the ZJU-FAST-Lab Fast-Drone-250 / EGO-Planner stack as an upstream dependency. The supplied project archive did not preserve the exact upstream Git commit, so this release does not invent a commit hash.

Only the project-specific files that were modified or added around the upstream planner are preserved under:

`third_party/fast-drone-250-modifications/overlay/`

The overlay keeps the original relative paths so it can be compared with, or applied to, a compatible upstream Fast-Drone-250 checkout.

## Confirmed project-specific planner changes

### 1. Replanning failsafe in `ego_replan_fsm.cpp`

The modified FSM adds real-flight failure handling around replanning. When replanning fails, the code can publish a HOLD trajectory at the current odometry position rather than simply continuing an obsolete trajectory.

Relevant parameters include:

- `fsm/enable_hold_on_replan_failure`
- `fsm/hold_replan_min_interval`

### 2. Goal-height validation

Manual/external goals are checked before entering the global trajectory and may be clamped into a configured vertical safety interval.

Relevant parameters include:

- `fsm/enable_target_z_clamp`
- `fsm/target_z_default`
- `fsm/min_target_z`
- `fsm/max_target_z`

### 3. Odometry-based replanning start

The modified FSM can use the measured odometry state as the replanning start state when the executed vehicle state deviates from the old B-spline trajectory.

Relevant parameters include:

- `fsm/replan_start_from_odom`
- `fsm/replan_odom_error_threshold`

### 4. Future 3D body-envelope collision check

The FSM contains a sampled future-trajectory safety check that tests a configurable 3D vehicle envelope instead of treating the UAV only as a mathematical point.

Relevant parameters include:

- `fsm/enable_future_body_check`
- `fsm/future_body_check_dt`
- `fsm/future_body_radius`
- `fsm/future_body_up_clearance`
- `fsm/future_body_down_clearance`
- `fsm/future_body_z_step`
- `fsm/future_body_circle_samples`

### 5. Post-planning trajectory-height filter

`planner_manager.cpp` adds a final B-spline validation stage before trajectory publication/execution. It samples the planned trajectory and rejects a solution whose altitude falls outside the allowed interval.

Relevant parameters include:

- `manager/enable_traj_height_filter`
- `manager/min_traj_z`
- `manager/max_traj_z`
- `manager/traj_height_check_dt`
- `manager/traj_height_tolerance`

### 6. Post-planning 3D body-clearance filter

`planner_manager.cpp` also adds a trajectory-level body-envelope validation layer. A B-spline can be rejected if sampled envelope points intersect inflated obstacles.

Relevant parameters include:

- `manager/enable_traj_body_filter`
- `manager/traj_body_check_dt`
- `manager/traj_body_radius`
- `manager/traj_body_up_clearance`
- `manager/traj_body_down_clearance`
- `manager/traj_body_z_step`
- `manager/traj_body_circle_samples`

### 7. Real-flight ROS integration

The project-specific launch files adapt the planner to the deployed ROS/PX4 stack, including MAVROS odometry and Livox/FAST-LIO point-cloud topics.

Preserved files:

- `advanced_param_exp.xml`
- `single_run_in_exp.launch`
- `single_run_in_vision.launch`
- `single_run_in_fox.launch`
- `default.rviz`

## Custom `fox_controller`

`uav/ros_ws/src/fox_controller/` is kept separately because it is a project-specific ROS package rather than an upstream EGO-Planner module.

It implements UAV mission execution around PX4/MAVROS, including sequential waypoint goals, per-waypoint hover timing, RC-triggered takeoff/landing, setpoint publication, mission-state handling, and `STOP_TASK` coordination with the higher-level mission supervisor.

## Attribution rule

When publishing this repository:

- describe EGO/Fast-Drone-250 as an upstream dependency;
- describe the files in this overlay as **modified upstream files**;
- describe `fox_controller` and the project's other custom ROS nodes as project code;
- preserve any upstream copyright/license notices when later syncing with a complete upstream checkout.

For a reproducible public fork, the best long-term approach is to create a GitHub fork of the exact upstream repository and re-apply these changes there, once the original base commit can be identified.
