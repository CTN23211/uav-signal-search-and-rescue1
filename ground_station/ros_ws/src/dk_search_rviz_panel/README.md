# DK Search RViz Panel - Region + RF Gradient

This panel is for the region-only search mode:

- The task boundary is the last-seen/search area.
- No last-known point is required.
- It subscribes to `/rf_gradient_status` and shows RF mode/confidence/trend.
- It includes `/rf_gradient_marker` in the RViz config.
- `RF -> Main` publishes `/main_routes_refresh`.
- `RF -> Backup` publishes `/backup_routes_refresh`.
- `Send Route` publishes `/send_route_to_uav` for the route bridge.

Install:

```bash
cd $HOME/catkin_ws/src
rm -rf dk_search_rviz_panel
unzip /mnt/data/dk_search_rviz_panel_region_rf_gradient.zip
cd $HOME/catkin_ws
catkin_make
source devel/setup.bash
roslaunch dk_search_rviz_panel search_panel.launch
```
