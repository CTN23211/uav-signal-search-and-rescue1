# Ground search-planner version history

The project evolved through multiple large `uav_3d_search_fusion_*` variants during field integration. Historical filenames recovered from prior work include examples such as:

- `uav_3d_search_fusion_with_tcp_route_send.py`
- `uav_3d_search_fusion_with_calib_indicator.py`
- `uav_3d_search_fusion_initial_center_yaw_locked.py`
- `uav_3d_search_fusion_initial_center_yaw_locked_lora5_ready.py`
- `uav_3d_search_fusion_lora5_logic_fixed.py`
- `uav_3d_search_fusion_lora5_logic_fixed_lr_swap.py`
- `uav_3d_search_fusion_source_prob_refactor.py`
- `uav_3d_search_fusion_startup_only_no_continuous_gradient.py`
- `uav_3d_search_fusion_10s_full_rebuild.py`

V3 deliberately does **not** publish all of those side by side.

The canonical V3 planner is:

```text
ground_station/ros_ws/src/dk_map_builder/scripts/uav_3d_search_fusion.py
```

It was taken from the recovered refactored DK2500 catkin workspace, where it matched the packaged `initial_center_yaw_locked_lora5_ready` copy and had already absorbed the later startup calibration, RF/source-probability, stable-route and route-sender logic.

Historical variants should be recovered from source backups / Git history only when debugging regression history.
