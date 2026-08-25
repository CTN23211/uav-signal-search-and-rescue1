#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import re
import time
import struct
import socket
import threading
from collections import deque

import rospy
import numpy as np

from std_msgs.msg import String, Header, Bool
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2


class UAV3DSearchFusionNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")

        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_from_jetson")
        self.lora_topic = rospy.get_param("~lora_topic", "/lora_from_jetson")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_from_jetson")

        self.map_size = rospy.get_param("~map_size", 30.0)
        self.resolution = rospy.get_param("~resolution", 0.2)
        self.origin_x = rospy.get_param("~origin_x", -self.map_size / 2.0)
        self.origin_y = rospy.get_param("~origin_y", -self.map_size / 2.0)

        self.width = int(self.map_size / self.resolution)
        self.height = int(self.map_size / self.resolution)

        # 最近点云窗口：用于实时更新障碍地图
        # 这里不再把 /uav_recent_obstacle_cloud 当作单帧点云。
        # 默认采用“目标点数 + 最大时间窗口”的滚动累计：尽量累积到 recent_target_points，
        # 同时不超过 recent_max_window_sec，最后通过体素下采样与 max_recent_points 控制规模。
        self.recent_window_sec = rospy.get_param("~recent_window_sec", 20.0)  # 兼容旧参数；adaptive=false 时使用
        self.recent_max_window_sec = rospy.get_param("~recent_max_window_sec", 60.0)
        self.recent_min_window_sec = rospy.get_param("~recent_min_window_sec", 0.0)
        self.recent_target_points = rospy.get_param("~recent_target_points", 50000)
        self.recent_adaptive_accumulation = rospy.get_param("~recent_adaptive_accumulation", True)
        self.recent_voxel_size = rospy.get_param("~recent_voxel_size", 0.02)
        self.max_recent_points = rospy.get_param("~max_recent_points", 60000)
        self.recent_cloud_topic = rospy.get_param("~recent_cloud_topic", "/uav_recent_obstacle_cloud")
        self.last_recent_raw_points_seen = 0
        self.last_recent_frames_used = 0
        self.last_recent_oldest_age = 0.0
        self.last_recent_downsampled_points = 0

        # 距离分层累计：近处少累计、远处多累计。
        # 目的不是伪造远处点，而是限制近处密集点占满缓存，给远处稀疏点更多累计时间和点数预算。
        self.enable_range_weighted_recent = rospy.get_param("~enable_range_weighted_recent", True)
        self.range_near_dist = rospy.get_param("~range_near_dist", 3.0)
        self.range_mid_dist = rospy.get_param("~range_mid_dist", 8.0)
        self.range_far_dist = rospy.get_param("~range_far_dist", 15.0)

        # 不同距离层的累计时间上限：近处短，远处长。
        self.range_near_max_age = rospy.get_param("~range_near_max_age", 5.0)
        self.range_mid_max_age = rospy.get_param("~range_mid_max_age", 18.0)
        self.range_far_max_age = rospy.get_param("~range_far_max_age", 60.0)
        self.range_very_far_max_age = rospy.get_param("~range_very_far_max_age", 60.0)

        # 不同距离层的点数预算比例：近处少，远处多。比例会自动归一化。
        self.range_near_quota_ratio = rospy.get_param("~range_near_quota_ratio", 0.15)
        self.range_mid_quota_ratio = rospy.get_param("~range_mid_quota_ratio", 0.25)
        self.range_far_quota_ratio = rospy.get_param("~range_far_quota_ratio", 0.35)
        self.range_very_far_quota_ratio = rospy.get_param("~range_very_far_quota_ratio", 0.25)

        # 不同距离层的体素大小：近处体素大，远处体素小。
        self.range_near_voxel = rospy.get_param("~range_near_voxel", 0.06)
        self.range_mid_voxel = rospy.get_param("~range_mid_voxel", 0.035)
        self.range_far_voxel = rospy.get_param("~range_far_voxel", 0.02)
        self.range_very_far_voxel = rospy.get_param("~range_very_far_voxel", 0.015)

        # 当远处点不足时，是否允许近处继续填满总点数。默认关闭，避免近处重新占满缓存。
        self.range_allow_near_overflow = rospy.get_param("~range_allow_near_overflow", False)
        self.last_recent_range_counts = [0, 0, 0, 0]
        self.last_recent_range_raw_seen = [0, 0, 0, 0]

        # 已观测自由空间：用于避免把墙外/未知区域误判为高回报区域
        # free_voxels 由无人机当前位置到障碍点的射线生成，表示激光确认穿过的空域
        self.enable_observed_filter = rospy.get_param("~enable_observed_filter", True)
        self.observed_free_cloud_topic = rospy.get_param("~observed_free_cloud_topic", "/uav_observed_free_cloud")
        self.free_ray_step = rospy.get_param("~free_ray_step", 0.20)
        self.free_ray_stop_before = rospy.get_param("~free_ray_stop_before", 0.25)
        self.observed_neighbor_radius = rospy.get_param("~observed_neighbor_radius", 0.45)
        self.observed_unknown_score = rospy.get_param("~observed_unknown_score", 0.0)

        # 无人机候选搜索高度
        self.search_z_min = rospy.get_param("~search_z_min", 2.5)
        self.search_z_max = rospy.get_param("~search_z_max", 4.5)
        self.candidate_step_xy = rospy.get_param("~candidate_step_xy", 0.3)
        self.candidate_step_z = rospy.get_param("~candidate_step_z", 0.3)

        # 障碍物点云参与范围
        self.obstacle_z_min = rospy.get_param("~obstacle_z_min", -2.0)
        self.obstacle_z_max = rospy.get_param("~obstacle_z_max", 8.0)

        # 无人机安全距离
        self.hard_collision_radius = rospy.get_param("~hard_collision_radius", 0.45)
        self.safe_radius = rospy.get_param("~safe_radius", 1.2)
        self.obstacle_voxel_size = rospy.get_param("~obstacle_voxel_size", 0.15)

        # 墙体/立面识别参数
        self.wall_min_points = rospy.get_param("~wall_min_points", 8)
        self.wall_min_z_span = rospy.get_param("~wall_min_z_span", 1.0)
        self.wall_inflation_radius = rospy.get_param("~wall_inflation_radius", 0.6)

        # 地形约束层：由滑动累计点云生成 2.5D DEM/坡度/起伏评分
        self.enable_terrain_filter = rospy.get_param("~enable_terrain_filter", False)
        # 默认值按室内稀疏点云调宽：地形层只做软惩罚，不负责墙外硬过滤
        self.terrain_soft_filter = rospy.get_param("~terrain_soft_filter", True)
        self.terrain_min_multiplier = rospy.get_param("~terrain_min_multiplier", 1.0)
        self.terrain_unknown_score = rospy.get_param("~terrain_unknown_score", 1.0)
        self.max_walkable_slope_deg = rospy.get_param("~max_walkable_slope_deg", 60.0)
        self.cliff_slope_deg = rospy.get_param("~cliff_slope_deg", 85.0)
        self.max_ground_height_span = rospy.get_param("~max_ground_height_span", 3.0)
        self.min_terrain_points = rospy.get_param("~min_terrain_points", 2)
        self.human_z_min = rospy.get_param("~human_z_min", 0.0)
        self.human_z_max = rospy.get_param("~human_z_max", 2.3)
        self.human_z_soft_max = rospy.get_param("~human_z_soft_max", 3.0)

        # 视线遮挡检测
        self.enable_line_of_sight = rospy.get_param("~enable_line_of_sight", True)
        self.los_step = rospy.get_param("~los_step", 0.15)
        self.los_skip_near = rospy.get_param("~los_skip_near", 0.4)

        # 飞机前向视场过滤：背后盲区降分
        self.use_fov_filter = rospy.get_param("~use_fov_filter", True)
        self.front_fov_deg = rospy.get_param("~front_fov_deg", 120.0)
        self.soft_fov_deg = rospy.get_param("~soft_fov_deg", 180.0)
        self.back_score = rospy.get_param("~back_score", 0.0)
        self.yaw_offset_deg = rospy.get_param("~yaw_offset_deg", 0.0)
        self.yaw_offset = math.radians(self.yaw_offset_deg)

        self.max_input_points_per_frame = rospy.get_param("~max_input_points_per_frame", 100000)
        self.max_obstacle_points = rospy.get_param("~max_obstacle_points", 60000)
        # free space 射线构建最耗时，最近障碍点云可以 5 万点，但射线不必对所有点都做。
        self.max_free_ray_points = rospy.get_param("~max_free_ray_points", 20000)
        self.max_score_points = rospy.get_param("~max_score_points", 50000)

        self.cloud_process_interval = rospy.get_param("~cloud_process_interval", 0.1)

        # 全地图重建节流：云回调只缓存点云帧，障碍/墙体/free space/terrain/score/路线默认按周期统一重建。
        # 这样可以避免每来一帧点云就重建整张地图，降低 DK2500 端 CPU 压力。
        self.full_map_rebuild_interval = rospy.get_param("~full_map_rebuild_interval", 10.0)
        self.rebuild_map_only_on_interval = rospy.get_param("~rebuild_map_only_on_interval", True)
        self.full_map_rebuild_refresh_routes = rospy.get_param("~full_map_rebuild_refresh_routes", False)
        self.score_publish_interval = rospy.get_param(
            "~score_publish_interval",
            self.full_map_rebuild_interval if self.rebuild_map_only_on_interval else 1.0
        )

        self.min_goal_score = rospy.get_param("~min_goal_score", 0.05)
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 0.8)

        # 目标点稳定策略：避免 next_search_goal 在多个相近高分点之间跳变
        self.goal_cluster_top_k = rospy.get_param("~goal_cluster_top_k", 120)
        self.goal_cluster_ratio = rospy.get_param("~goal_cluster_ratio", 0.85)
        self.goal_distance_penalty = rospy.get_param("~goal_distance_penalty", 0.25)
        self.goal_jump_penalty = rospy.get_param("~goal_jump_penalty", 0.60)
        self.goal_hold_time = rospy.get_param("~goal_hold_time", 3.0)
        self.goal_switch_ratio = rospy.get_param("~goal_switch_ratio", 1.20)
        self.goal_smooth_alpha = rospy.get_param("~goal_smooth_alpha", 0.35)
        self.max_goal_step = rospy.get_param("~max_goal_step", 1.5)

        self.current_goal = None
        self.current_goal_value = 0.0
        self.current_goal_raw_score = 0.0
        self.current_goal_time = 0.0

        # 高分目标点序列输出：用于把所有高收益搜索视点组织成路径点
        self.high_goal_max_num = rospy.get_param("~high_goal_max_num", 20)
        self.high_goal_score_ratio = rospy.get_param("~high_goal_score_ratio", 0.75)
        self.high_goal_min_score = rospy.get_param("~high_goal_min_score", 0.08)
        self.high_goal_min_separation = rospy.get_param("~high_goal_min_separation", 1.0)
        self.path_order_mode = rospy.get_param("~path_order_mode", "nearest")  # nearest 或 score
        self.path_include_current_position = rospy.get_param("~path_include_current_position", True)

        self.high_goals_topic = rospy.get_param("~high_goals_topic", "/high_score_goals")
        self.high_goals_cloud_topic = rospy.get_param("~high_goals_cloud_topic", "/high_score_goals_cloud")
        self.waypoints_topic = rospy.get_param("~waypoints_topic", "/uav_search_waypoints")
        self.waypoint_path_topic = rospy.get_param("~waypoint_path_topic", "/uav_search_waypoints_path")

        # 显式有序 goal 阵列输出：PoseArray 的 poses[0], poses[1], ... 就是执行顺序。
        # 同时输出 JSON 序列和 RViz 编号 Marker，方便控制节点读取与可视化核对。
        self.ordered_goal_array_topic = rospy.get_param("~ordered_goal_array_topic", "/ordered_goal_array")
        self.ordered_goal_path_topic = rospy.get_param("~ordered_goal_path_topic", "/ordered_goal_path")
        self.ordered_goal_markers_topic = rospy.get_param("~ordered_goal_markers_topic", "/ordered_goal_markers")
        self.ordered_goal_sequence_topic = rospy.get_param("~ordered_goal_sequence_topic", "/ordered_goal_sequence")
        # 任务航线可视化默认把当前位置 START 加入 /ordered_goal_path 和 /ordered_goal_markers，
        # 这样 RViz 中会明确显示 START -> 入口/第一个搜索点这一段。
        self.ordered_goal_include_current_position = rospy.get_param("~ordered_goal_include_current_position", True)
        # 为避免控制器误把当前位置当作第一个要飞的搜索点，/ordered_goal_array 默认不加入当前位置。
        self.ordered_goal_array_include_current_position = rospy.get_param("~ordered_goal_array_include_current_position", False)

        # RViz 可视化增强：PoseArray/Path 本身看不出编号，具体顺序靠 MarkerArray 显示。
        # lifetime=0 表示 marker 不自动消失，避免 RViz 里闪烁或刚添加 Display 时看不到。
        self.ordered_goal_marker_lifetime = rospy.get_param("~ordered_goal_marker_lifetime", 0.0)
        self.ordered_goal_point_scale = rospy.get_param("~ordered_goal_point_scale", 0.35)
        self.ordered_goal_text_scale = rospy.get_param("~ordered_goal_text_scale", 0.65)
        self.ordered_goal_text_z_offset = rospy.get_param("~ordered_goal_text_z_offset", 0.85)
        self.ordered_goal_line_width = rospy.get_param("~ordered_goal_line_width", 0.08)
        self.ordered_goal_arrow_enabled = rospy.get_param("~ordered_goal_arrow_enabled", True)
        self.ordered_goal_arrow_shaft = rospy.get_param("~ordered_goal_arrow_shaft", 0.06)
        self.ordered_goal_arrow_head = rospy.get_param("~ordered_goal_arrow_head", 0.18)
        self.ordered_goal_arrow_head_len = rospy.get_param("~ordered_goal_arrow_head_len", 0.25)
        self.ordered_goal_show_score = rospy.get_param("~ordered_goal_show_score", True)
        self.ordered_goal_show_start_marker = rospy.get_param("~ordered_goal_show_start_marker", True)

        # 主线路 + 备用探索线路。
        # 主线路仍由 /ordered_goal_array、/ordered_goal_path、/ordered_goal_markers 输出；
        # 备用线路作为分叉探索路线输出到 /backup_goal_markers 和 /backup_goal_sequence。
        self.branch_enable = rospy.get_param("~branch_enable", True)
        self.branch_max_routes = rospy.get_param("~branch_max_routes", 3)
        self.branch_max_routes_per_attach = rospy.get_param("~branch_max_routes_per_attach", 2)
        self.branch_max_len = rospy.get_param("~branch_max_len", 4)
        self.branch_candidate_pool = rospy.get_param("~branch_candidate_pool", 800)
        self.branch_attach_radius = rospy.get_param("~branch_attach_radius", 4.0)
        self.branch_extend_radius = rospy.get_param("~branch_extend_radius", 2.2)
        self.branch_min_off_main_dist = rospy.get_param("~branch_min_off_main_dist", 0.8)
        self.branch_min_angle_deg = rospy.get_param("~branch_min_angle_deg", 35.0)
        self.branch_score_ratio = rospy.get_param("~branch_score_ratio", 0.20)
        self.branch_min_score = rospy.get_param("~branch_min_score", 0.005)
        self.branch_min_separation = rospy.get_param("~branch_min_separation", 0.7)
        self.branch_route_discount = rospy.get_param("~branch_route_discount", 0.85)
        self.branch_include_attach_point = rospy.get_param("~branch_include_attach_point", True)

        # 备用线路输出模式：
        # True  表示备用线路是“主线前缀 + 分叉支线”，例如 01 -> 02 -> 03 -> B1-01 -> B1-02；
        # False 表示只显示/切换“分叉点 + 支线”，例如 03 -> B1-01 -> B1-02。
        self.branch_include_main_prefix = rospy.get_param("~branch_include_main_prefix", True)
        self.branch_draw_main_prefix = rospy.get_param("~branch_draw_main_prefix", True)

        # 备用线路挂载约束：避免 RViz 中出现“没挂到主线路节点”的孤立备用线。
        # branch_require_attached=True 时，每条 B 路线都必须有 branch_from_goal、main_prefix_goals，
        # 并且第一个 Bx-01 距离挂载主节点不能超过 branch_max_first_branch_dist。
        self.branch_require_attached = rospy.get_param("~branch_require_attached", True)
        self.branch_max_first_branch_dist = rospy.get_param("~branch_max_first_branch_dist", self.branch_attach_radius)
        # >0 时强制所有备用路线只能挂到指定主线路序号，例如 3 表示只能从 03 分叉。
        # 0 表示自动选择挂载节点。
        self.branch_force_attach_seq = rospy.get_param("~branch_force_attach_seq", 0)
        # 在备用线路 Marker 中把挂载主节点画成一个明显的方块，防止只有文字导致看起来没挂上。
        self.branch_draw_attach_sphere = rospy.get_param("~branch_draw_attach_sphere", True)

        self.branch_use_xy_angle = rospy.get_param("~branch_use_xy_angle", True)
        self.backup_goal_markers_topic = rospy.get_param("~backup_goal_markers_topic", "/backup_goal_markers")
        self.backup_goal_sequence_topic = rospy.get_param("~backup_goal_sequence_topic", "/backup_goal_sequence")
        self.select_backup_route_topic = rospy.get_param("~select_backup_route_topic", "/select_backup_route")
        self.backup_goal_marker_lifetime = rospy.get_param("~backup_goal_marker_lifetime", 0.0)
        self.backup_goal_point_scale = rospy.get_param("~backup_goal_point_scale", 0.28)
        self.backup_goal_text_scale = rospy.get_param("~backup_goal_text_scale", 0.52)
        self.backup_goal_text_z_offset = rospy.get_param("~backup_goal_text_z_offset", 0.75)
        self.backup_goal_line_width = rospy.get_param("~backup_goal_line_width", 0.055)

        # 可选：向 /select_backup_route 发布 std_msgs/String，例如 data: "B1"，
        # 下一周期会把主队列替换为“分叉点 + B1 线路”。
        self.enable_backup_route_selection = rospy.get_param("~enable_backup_route_selection", True)
        self.selected_backup_route_id = None
        self.last_backup_routes = []

        # 主线路 + 备用线路手动刷新模式。
        # True 时：主线路和备用线路不会随 score_publish_interval 每秒重算。
        # 主线路刷新和备用线路刷新使用两个不同命令：
        #   /main_routes_refresh   只刷新主线路，不改变备用线路缓存；
        #   /backup_routes_refresh 只刷新备用线路，基于当前稳定主线路生成；
        # 兼容保留 /search_routes_refresh，用于“一次性同时刷新主线路和备用线路”。
        self.routes_manual_refresh = rospy.get_param("~routes_manual_refresh", True)
        self.main_routes_refresh_topic = rospy.get_param("~main_routes_refresh_topic", "/main_routes_refresh")
        self.backup_routes_refresh_topic = rospy.get_param("~backup_routes_refresh_topic", "/backup_routes_refresh")
        self.routes_refresh_topic = rospy.get_param("~routes_refresh_topic", "/search_routes_refresh")
        self.backup_routes_manual_refresh = rospy.get_param("~backup_routes_manual_refresh", self.routes_manual_refresh)
        self.backup_routes_refresh_requested = True
        self.cached_backup_routes = []
        self.cached_backup_valid = False
        self.cached_backup_main_goals = []
        self.cached_backup_threshold = 0.0
        self.cached_backup_max_score = 0.0
        self.cached_backup_time = 0.0
        self.cached_backup_action = "init"

        # 有序 goal 队列稳定策略：避免 /ordered_goal_array 每秒整体重排。
        # 默认采用严格锁存模式：生成一次队列后保持不变，只有外部确认到达、手动重置、队列为空时才更新。
        self.ordered_goal_queue_enable = rospy.get_param("~ordered_goal_queue_enable", True)
        self.ordered_goal_strict_lock = rospy.get_param("~ordered_goal_strict_lock", True)

        # 自动到达删除默认关闭，防止无人机离第一个 goal 很近时编号频繁前移。
        # 更推荐由控制器发布 /ordered_goal_reached 来删除队首。
        self.ordered_goal_drop_reached = rospy.get_param("~ordered_goal_drop_reached", False)
        self.ordered_goal_reached_radius = rospy.get_param("~ordered_goal_reached_radius", 0.25)
        self.ordered_goal_reached_xy_radius = rospy.get_param("~ordered_goal_reached_xy_radius", self.ordered_goal_reached_radius)
        self.ordered_goal_reached_z_radius = rospy.get_param("~ordered_goal_reached_z_radius", 0.35)
        self.ordered_goal_reached_hold_time = rospy.get_param("~ordered_goal_reached_hold_time", 1.0)

        # 严格锁存模式下默认几乎不自动重规划。需要刷新时发 /ordered_goal_reset。
        self.ordered_goal_min_replan_interval = rospy.get_param("~ordered_goal_min_replan_interval", 9999.0)
        self.ordered_goal_force_replan_interval = rospy.get_param("~ordered_goal_force_replan_interval", 9999.0)
        self.ordered_goal_allow_force_replan_in_lock = rospy.get_param("~ordered_goal_allow_force_replan_in_lock", False)
        self.ordered_goal_replan_score_ratio = rospy.get_param("~ordered_goal_replan_score_ratio", 10.0)
        self.ordered_goal_sequence_discount = rospy.get_param("~ordered_goal_sequence_discount", 0.90)
        self.ordered_goal_allow_empty_replan = rospy.get_param("~ordered_goal_allow_empty_replan", False)

        # 外部控制接口：控制器确认到达后发布 /ordered_goal_reached；需要重新生成路径时发布 /ordered_goal_reset。
        self.ordered_goal_reached_topic = rospy.get_param("~ordered_goal_reached_topic", "/ordered_goal_reached")
        self.ordered_goal_reset_topic = rospy.get_param("~ordered_goal_reset_topic", "/ordered_goal_reset")

        # DK2500 -> Jetson Mission Supervisor 路线发送功能。
        # 本节点直接把当前稳定主路线 stable_ordered_goals 打包为 JSON，
        # 通过 TCP 发送到 Jetson 端 uav_mission_supervisor 的 TCP Server。
        self.enable_tcp_route_sender = rospy.get_param("~enable_tcp_route_sender", True)
        self.jetson_mission_ip = rospy.get_param("~jetson_mission_ip", "192.168.1.20")
        self.jetson_mission_port = int(rospy.get_param("~jetson_mission_port", 9100))
        self.route_send_trigger_topic = rospy.get_param("~route_send_trigger_topic", "/send_route_to_uav")
        self.route_send_mission_cmd_topic = rospy.get_param("~route_send_mission_cmd_topic", "/dk_tcp_mission_cmd")
        self.route_send_status_topic = rospy.get_param("~route_send_status_topic", "/dk_tcp_sender_status")
        self.route_send_route_type = rospy.get_param("~route_send_route_type", "MAIN_ROUTE")
        self.route_send_frame_id = rospy.get_param("~route_send_frame_id", self.frame_id)
        self.route_send_hold_sec = float(rospy.get_param("~route_send_hold_sec", 2.0))
        self.route_send_max_goals = int(rospy.get_param("~route_send_max_goals", 30))
        self.route_send_force_z = rospy.get_param("~route_send_force_z", False)
        self.route_send_default_z = float(rospy.get_param("~route_send_default_z", 2.0))
        self.route_send_min_valid_z = float(rospy.get_param("~route_send_min_valid_z", 0.2))
        self.route_send_max_valid_z = float(rospy.get_param("~route_send_max_valid_z", 5.0))
        self.route_send_include_entry = rospy.get_param("~route_send_include_entry", False)
        # 发送给无人机的路线可追加原路返回段：
        # 例如 01 -> 02 -> 03 会扩展为 01 -> 02 -> 03 -> 02 -> 01 -> START。
        # START 为下发路线瞬间的无人机 odom 位置，便于飞机按原路回到出发附近。
        self.route_send_append_return_path = rospy.get_param("~route_send_append_return_path", True)
        self.route_send_return_to_current_position = rospy.get_param("~route_send_return_to_current_position", True)
        self.route_send_return_hold_sec = float(rospy.get_param("~route_send_return_hold_sec", self.route_send_hold_sec))
        self.route_send_return_route_types = str(rospy.get_param(
            "~route_send_return_route_types",
            "MAIN_ROUTE,LOCAL_SEARCH_ROUTE"
        ))
        self.route_send_connect_timeout = float(rospy.get_param("~route_send_connect_timeout", 2.0))
        self.route_send_send_timeout = float(rospy.get_param("~route_send_send_timeout", 2.0))

        # OpenVINO/YOLO 识别事件接入任务规划器：
        # 订阅 /dk_target_event，收到 TARGET_CONFIRM 后直接向 Jetson mission supervisor
        # 发送 TARGET_CONFIRM 命令，使飞机保持当前目标/当前位置悬停。
        self.enable_target_event_mission_bridge = rospy.get_param("~enable_target_event_mission_bridge", True)
        self.target_event_topic = rospy.get_param("~target_event_topic", "/dk_target_event")
        self.target_event_confirm_values = str(rospy.get_param(
            "~target_event_confirm_values",
            "TARGET_CONFIRM"
        ))
        self.target_event_suspect_values = str(rospy.get_param(
            "~target_event_suspect_values",
            "TARGET_SUSPECTED"
        ))
        self.target_event_lost_values = str(rospy.get_param(
            "~target_event_lost_values",
            "TARGET_LOST,NO_TARGET"
        ))
        self.target_confirm_cmd = str(rospy.get_param("~target_confirm_cmd", "TARGET_CONFIRM"))
        self.target_confirm_min_interval = float(rospy.get_param("~target_confirm_min_interval", 3.0))
        self.target_confirm_preempt = rospy.get_param("~target_confirm_preempt", True)
        self.last_target_confirm_send_time = 0.0
        self.last_target_event = ""

        self.stable_ordered_goals = []
        self.stable_ordered_goal_value = 0.0
        self.stable_ordered_goal_time = 0.0
        self.stable_ordered_goal_threshold = 0.0
        self.stable_ordered_goal_max_score = 0.0
        self.stable_ordered_goal_last_action = "init"
        self.ordered_goal_external_reached_requested = False
        self.ordered_goal_reset_requested = False
        self.ordered_goal_auto_reached_since = 0.0
        self.queue_lock = threading.Lock()

        # LoRa 参数
        self.min_snr = rospy.get_param("~min_snr", 0.0)
        self.target_prefix = rospy.get_param("~target_prefix", "HELP")
        self.rf_decay = rospy.get_param("~rf_decay", 0.995)
        self.rf_gain = rospy.get_param("~rf_gain", 0.45)

        # LoRa 小场地虚拟衰减参数。
        # 只改变算法用于 RF 热力图和梯度计算的 RSSI，不改变真实 LoRa 通信。
        # 例如 raw RSSI=-52, lora_virtual_attenuation_db=35 时，算法按 -87 dBm 参与计算。
        self.lora_virtual_attenuation_db = rospy.get_param("~lora_virtual_attenuation_db", 35.0)
        self.lora_rssi_score_min = rospy.get_param("~lora_rssi_score_min", -105.0)
        self.lora_rssi_score_max = rospy.get_param("~lora_rssi_score_max", -65.0)
        self.lora_snr_score_max = rospy.get_param("~lora_snr_score_max", 12.0)

        # 连续 RF 梯度默认关闭：避免“采几个点后一直输出梯度”。
        # 开局校准仍可通过 /startup_rf_calib_result 输出一次性 INIT RF 结果。
        self.enable_rf_gradient = rospy.get_param("~enable_rf_gradient", False)
        self.rf_gradient_window_sec = rospy.get_param("~rf_gradient_window_sec", 45.0)
        self.rf_gradient_min_samples = rospy.get_param("~rf_gradient_min_samples", 6)
        self.rf_gradient_min_spread = rospy.get_param("~rf_gradient_min_spread", 1.2)
        self.rf_gradient_min_motion = rospy.get_param("~rf_gradient_min_motion", 0.25)
        self.rf_gradient_conf_gain = rospy.get_param("~rf_gradient_conf_gain", 20.0)
        self.rf_gradient_conf_threshold = rospy.get_param("~rf_gradient_conf_threshold", 0.25)
        self.rf_gradient_weight = rospy.get_param("~rf_gradient_weight", 0.35)
        self.rf_gradient_min_multiplier = rospy.get_param("~rf_gradient_min_multiplier", 0.75)
        self.rf_gradient_max_multiplier = rospy.get_param("~rf_gradient_max_multiplier", 1.35)
        self.rf_signal_rise_threshold = rospy.get_param("~rf_signal_rise_threshold", 0.04)
        self.rf_signal_drop_threshold = rospy.get_param("~rf_signal_drop_threshold", -0.06)
        self.rf_refine_quality_threshold = rospy.get_param("~rf_refine_quality_threshold", 0.72)
        self.rf_gradient_affect_score = rospy.get_param("~rf_gradient_affect_score", True)
        self.rf_gradient_max_samples = rospy.get_param("~rf_gradient_max_samples", 2000)
        # 是否发布连续 RF 梯度状态/箭头。默认关闭，只保留开局校准的最终 INIT RF 可视化。
        self.publish_continuous_rf_gradient = rospy.get_param("~publish_continuous_rf_gradient", False)

        # 更鲁棒的 LoRa 信号源定位层：
        # 不再只依赖局部 RF 梯度，而是用最近一段时间的多点 LoRa 样本
        # 在任务区内构建“信号源存在概率图”。
        # 方法采用相对强弱排序：若样本 i 的 quality 高于样本 j，
        # 则候选源点更应该靠近 i 而远离 j。该方法不依赖绝对发射功率，
        # 对小场地虚拟衰减和 RSSI 标定误差更稳。
        self.enable_source_localization = rospy.get_param("~enable_source_localization", True)
        self.source_probability_topic = rospy.get_param("~source_probability_topic", "/source_probability_map")
        self.source_estimate_marker_topic = rospy.get_param("~source_estimate_marker_topic", "/source_estimate_marker")
        self.source_estimate_status_topic = rospy.get_param("~source_estimate_status_topic", "/source_estimate_status")
        self.source_local_route_trigger_topic = rospy.get_param("~source_local_route_trigger_topic", "/send_local_search_to_uav")

        self.source_update_interval = rospy.get_param("~source_update_interval", 2.0)
        self.source_window_sec = rospy.get_param("~source_window_sec", 180.0)
        self.source_min_samples = rospy.get_param("~source_min_samples", 5)
        self.source_max_samples = rospy.get_param("~source_max_samples", 60)
        self.source_pair_max_pairs = rospy.get_param("~source_pair_max_pairs", 2500)
        self.source_pair_min_quality_delta = rospy.get_param("~source_pair_min_quality_delta", 0.015)
        self.source_pair_gain = rospy.get_param("~source_pair_gain", 7.0)
        self.source_pair_distance_scale = rospy.get_param("~source_pair_distance_scale", 2.0)
        self.source_anchor_weight = rospy.get_param("~source_anchor_weight", 0.25)
        self.source_anchor_sigma = rospy.get_param("~source_anchor_sigma", 4.0)
        self.source_task_prior_weight = rospy.get_param("~source_task_prior_weight", 0.20)
        self.source_estimate_top_ratio = rospy.get_param("~source_estimate_top_ratio", 0.85)
        self.source_uncertainty_ratio = rospy.get_param("~source_uncertainty_ratio", 0.70)
        self.source_confidence_min_samples_full = rospy.get_param("~source_confidence_min_samples_full", 12)

        # 源概率图对路线的影响。source_score_weight 把源概率作为“搜索证据”，
        # source_multiplier_weight 再作为倍率提升概率峰附近候选点。
        self.source_affect_score = rospy.get_param("~source_affect_score", True)
        self.source_score_min_confidence = rospy.get_param("~source_score_min_confidence", 0.15)
        self.source_score_weight = rospy.get_param("~source_score_weight", 0.75)
        self.source_multiplier_weight = rospy.get_param("~source_multiplier_weight", 0.80)
        self.source_multiplier_min = rospy.get_param("~source_multiplier_min", 0.70)
        self.source_multiplier_max = rospy.get_param("~source_multiplier_max", 1.80)
        self.source_refine_confidence_threshold = rospy.get_param("~source_refine_confidence_threshold", 0.60)
        self.source_refine_uncertainty_radius = rospy.get_param("~source_refine_uncertainty_radius", 3.0)

        # 局部弓字形精搜路线参数。该路线可手动通过 /send_local_search_to_uav 触发，
        # 也可作为高置信源估计后的下一阶段路线。
        self.source_local_search_size = rospy.get_param("~source_local_search_size", 3.0)
        self.source_local_search_spacing = rospy.get_param("~source_local_search_spacing", 0.6)
        self.source_local_search_max_goals = rospy.get_param("~source_local_search_max_goals", 30)
        self.source_local_search_hold_sec = rospy.get_param("~source_local_search_hold_sec", 2.0)
        self.source_local_search_route_type = rospy.get_param("~source_local_search_route_type", "LOCAL_SEARCH_ROUTE")


        # 开局 LoRa 方向校准模块：在起飞开阔区自动选择一个局部安全校准中心，
        # 生成“前进式双横截面”采样路线，并把校准中心、采样点、初始 RF 方向发给 RViz。
        self.enable_startup_rf_calibration = rospy.get_param("~enable_startup_rf_calibration", True)
        self.startup_calib_start_topic = rospy.get_param("~startup_calib_start_topic", "/startup_rf_calib_start")
        self.startup_calib_reset_topic = rospy.get_param("~startup_calib_reset_topic", "/startup_rf_calib_reset")
        self.startup_calib_status_topic = rospy.get_param("~startup_calib_status_topic", "/startup_rf_calib_status")
        self.startup_calib_result_topic = rospy.get_param("~startup_calib_result_topic", "/startup_rf_calib_result")
        self.startup_calib_path_topic = rospy.get_param("~startup_calib_path_topic", "/startup_rf_calib_path")
        self.startup_calib_marker_topic = rospy.get_param("~startup_calib_marker_topic", "/startup_rf_calib_markers")

        self.startup_calib_auto_select_center = rospy.get_param("~startup_calib_auto_select_center", True)

        # 开局校准锚点锁定：
        # True 时，校准中心 P0/CENTER 固定为触发 /startup_rf_calib_start 瞬间的飞机 odom 位置；
        # 十字坐标轴的 forward 方向固定为触发瞬间的飞机 yaw。
        # 这样手动抱机校准时，RViz 中 P0 必然与飞机初始点重合，F/B/L/R 相对于初始机头方向展开。
        self.startup_calib_center_lock_to_initial_odom = rospy.get_param("~startup_calib_center_lock_to_initial_odom", True)
        self.startup_calib_axis_lock_to_initial_yaw = rospy.get_param("~startup_calib_axis_lock_to_initial_yaw", True)
        self.startup_calib_center_use_initial_odom_z = rospy.get_param("~startup_calib_center_use_initial_odom_z", True)

        self.startup_calib_search_radius = rospy.get_param("~startup_calib_search_radius", 8.0)
        self.startup_calib_candidate_step = rospy.get_param("~startup_calib_candidate_step", 2.0)
        self.startup_calib_open_radius = rospy.get_param("~startup_calib_open_radius", 4.0)
        self.startup_calib_min_clearance = rospy.get_param("~startup_calib_min_clearance", 1.5)
        self.startup_calib_vertical_margin = rospy.get_param("~startup_calib_vertical_margin", 1.2)
        self.startup_calib_preview_interval = rospy.get_param("~startup_calib_preview_interval", 2.0)

        # 校准路线默认是“小开阔区版”，外场足够大时可在 launch 里放大 forward/lateral。
        self.startup_calib_z = rospy.get_param("~startup_calib_z", 2.0)

        # 开局校准路线模式：
        #   centered_cross  : 先找一个开阔中心点，再以该点为圆心做十字采样。
        #   centered_sector : 先找一个开阔中心点，再以该点为圆心做扇形/圆周采样。
        #   forward_double_cross : 兼容旧版“前进式双横截面”采样。
        self.startup_calib_pattern = str(rospy.get_param("~startup_calib_pattern", "centered_cross")).lower()
        self.startup_calib_probe_radius = rospy.get_param("~startup_calib_probe_radius", 8.0)
        self.startup_calib_return_center_between_probes = rospy.get_param("~startup_calib_return_center_between_probes", True)
        self.startup_calib_include_diagonal_cross = rospy.get_param("~startup_calib_include_diagonal_cross", False)

        # centered_sector 模式参数。角度均相对于“校准中心 -> 任务区中心”的前向方向。
        # 若 startup_calib_sector_angles_deg 非空，则优先使用显式角度，例如 "-60,-30,0,30,60"。
        self.startup_calib_sector_count = rospy.get_param("~startup_calib_sector_count", 5)
        self.startup_calib_sector_span_deg = rospy.get_param("~startup_calib_sector_span_deg", 120.0)
        self.startup_calib_sector_angles_deg = str(rospy.get_param("~startup_calib_sector_angles_deg", ""))

        # 兼容旧版前进式双横截面参数。
        self.startup_calib_forward1 = rospy.get_param("~startup_calib_forward1", 12.0)
        self.startup_calib_forward2 = rospy.get_param("~startup_calib_forward2", 28.0)
        self.startup_calib_lateral1 = rospy.get_param("~startup_calib_lateral1", 6.0)
        self.startup_calib_lateral2 = rospy.get_param("~startup_calib_lateral2", 10.0)
        self.startup_calib_hold_sec = rospy.get_param("~startup_calib_hold_sec", 8.0)
        self.startup_calib_route_type = rospy.get_param("~startup_calib_route_type", "STARTUP_RF_CALIB_ROUTE")
        self.startup_calib_send_route_on_start = rospy.get_param("~startup_calib_send_route_on_start", True)

        self.startup_calib_expected_duration_sec = rospy.get_param("~startup_calib_expected_duration_sec", 120.0)
        self.startup_calib_min_duration_sec = rospy.get_param("~startup_calib_min_duration_sec", 35.0)
        self.startup_calib_min_samples = rospy.get_param("~startup_calib_min_samples", 6)
        self.startup_calib_min_spread = rospy.get_param("~startup_calib_min_spread", 4.0)
        self.startup_calib_conf_gain = rospy.get_param("~startup_calib_conf_gain", 18.0)
        self.startup_calib_conf_threshold = rospy.get_param("~startup_calib_conf_threshold", 0.25)
        self.startup_calib_auto_finish_on_conf = rospy.get_param("~startup_calib_auto_finish_on_conf", True)
        self.startup_calib_auto_refresh_main = rospy.get_param("~startup_calib_auto_refresh_main", True)
        self.startup_calib_auto_refresh_backup = rospy.get_param("~startup_calib_auto_refresh_backup", True)
        self.startup_calib_clear_rf_on_start = rospy.get_param("~startup_calib_clear_rf_on_start", True)
        self.startup_calib_use_as_initial_rf_prior = rospy.get_param("~startup_calib_use_as_initial_rf_prior", True)
        self.startup_calib_prior_hold_sec = rospy.get_param("~startup_calib_prior_hold_sec", 180.0)
        self.startup_calib_arrow_length = rospy.get_param("~startup_calib_arrow_length", 8.0)
        self.startup_calib_sample_assignment_radius = rospy.get_param("~startup_calib_sample_assignment_radius", 4.0)

        # 开局校准专用 LoRa 样本缓存。
        # 逻辑修正：开局校准要求 P0/F/B/L/R 都采够 LoRa 时，不能再复用连续 RF 梯度的滚动窗口，
        # 否则手动校准时间一长，早期点位样本会被 expected_duration_sec*2 的窗口清掉。
        self.startup_calib_max_samples = int(rospy.get_param("~startup_calib_max_samples", 5000))
        self.startup_calib_keep_all_active_samples = rospy.get_param("~startup_calib_keep_all_active_samples", True)

        # 开局 INIT RF 的 LoRa 点位采样门槛：
        # 只有指定的校准点都积攒到足够数量的 LoRa 样本后，才允许计算/显示 INIT RF，
        # 并且才允许该 INIT RF 作为后续路径规划的初始无线方向先验。
        self.startup_calib_required_lora_labels = rospy.get_param(
            "~startup_calib_required_lora_labels",
            "P0,F,B,L,R"
        )
        self.startup_calib_min_lora_per_point = int(rospy.get_param(
            "~startup_calib_min_lora_per_point",
            2
        ))
        self.startup_calib_require_lora_per_point_for_init = rospy.get_param(
            "~startup_calib_require_lora_per_point_for_init",
            True
        )
        self.startup_calib_require_lora_per_point_to_finish = rospy.get_param(
            "~startup_calib_require_lora_per_point_to_finish",
            True
        )

        # 开局校准点停留指示器：
        # 用 odom 判断无人机/手持传感器是否在每个校准点附近连续停够时间。
        # 满足条件的点在 RViz 中标绿；所有点均满足后输出 ALL CALIB POINTS READY。
        self.startup_calib_indicator_enable = rospy.get_param("~startup_calib_indicator_enable", True)
        self.startup_calib_indicator_topic = rospy.get_param("~startup_calib_indicator_topic", "/startup_rf_calib_indicator")
        self.startup_calib_indicator_required_sec = rospy.get_param(
            "~startup_calib_indicator_required_sec",
            self.startup_calib_hold_sec
        )
        self.startup_calib_indicator_radius = rospy.get_param("~startup_calib_indicator_radius", 0.60)
        self.startup_calib_indicator_check_z = rospy.get_param("~startup_calib_indicator_check_z", False)
        self.startup_calib_indicator_z_tolerance = rospy.get_param("~startup_calib_indicator_z_tolerance", 0.60)
        # 若置为 true，校准流程只有在所有校准点都变绿后才允许结束；默认不改变原有结束逻辑。
        self.startup_calib_require_all_points_ready_to_finish = rospy.get_param(
            "~startup_calib_require_all_points_ready_to_finish",
            False
        )

        self.rf_samples = deque(maxlen=int(self.rf_gradient_max_samples))
        self.rf_gradient_x = 0.0
        self.rf_gradient_y = 0.0
        self.rf_gradient_conf = 0.0
        self.rf_gradient_trend = 0.0
        self.rf_latest_quality = 0.0
        self.rf_search_mode = "COVERAGE"
        self.last_best_rf_sample = None


        # 开局 RF 校准状态。route_points: [{label,x,y,z,hold,kind}, ...]
        self.startup_calib_active = False
        self.startup_calib_completed = False
        self.startup_calib_start_time = 0.0
        self.startup_calib_finish_time = 0.0
        self.startup_calib_center = None
        self.startup_calib_center_score = 0.0
        self.startup_calib_center_reason = "NOT_SELECTED"

        # /startup_rf_calib_start 触发瞬间的飞机初始位姿。
        # 后续 CENTER/P0 和十字轴方向均可锁定到该初始位姿，避免预览或自动选点导致中心偏移。
        self.startup_calib_initial_x = None
        self.startup_calib_initial_y = None
        self.startup_calib_initial_z = None
        self.startup_calib_initial_yaw = None
        self.startup_calib_initial_pose_time = 0.0

        self.startup_calib_route_points = []
        self.startup_calib_last_preview_time = 0.0
        self.startup_calib_result = None
        # 专用于本次开局校准的 LoRa 样本缓存；不受连续 RF 梯度窗口裁剪影响。
        self.startup_calib_samples = deque(maxlen=max(100, int(self.startup_calib_max_samples)))
        self.startup_calib_refresh_sent = False
        self.startup_calib_last_status = {
            "state": "IDLE",
            "reason": "waiting_for_start",
        }
        self.startup_calib_point_states = []
        self.startup_calib_point_state_route_signature = ""
        self.startup_calib_all_points_ready = False
        self.startup_calib_last_indicator_update_time = 0.0

        # 先验区域，可不开
        self.use_prior = rospy.get_param("~use_prior", False)
        self.prior_x = rospy.get_param("~prior_x", 0.0)
        self.prior_y = rospy.get_param("~prior_y", 0.0)
        self.prior_sigma = rospy.get_param("~prior_sigma", 8.0)

        # 搜索任务输入：搜索边界、目标 ID、最后已知坐标。
        # 可通过参数初始化，也可运行时向 /set_search_task 发布 JSON 动态更新。
        self.search_task_topic = rospy.get_param("~search_task_topic", "/set_search_task")
        self.clear_search_task_topic = rospy.get_param("~clear_search_task_topic", "/clear_search_task")
        self.task_area_markers_topic = rospy.get_param("~task_area_markers_topic", "/task_area_markers")
        self.initial_probability_topic = rospy.get_param("~initial_probability_topic", "/initial_probability_map")
        self.search_task_status_topic = rospy.get_param("~search_task_status_topic", "/search_task_status")

        self.task_enabled = rospy.get_param("~task_enabled", False)
        self.target_id = str(rospy.get_param("~target_id", ""))
        self.task_has_last_known = rospy.get_param("~task_has_last_known", False)

        # 区域模式：任务片区就是最后消失/最后出现区域。
        # 不再使用单个 last_known 点，也不再画红色 Last known marker。
        self.task_region_only_mode = rospy.get_param("~task_region_only_mode", True)
        self.task_draw_last_known_marker = rospy.get_param("~task_draw_last_known_marker", False)
        if self.task_region_only_mode:
            self.task_has_last_known = False

        self.last_known_x = rospy.get_param("~last_known_x", 0.0)
        self.last_known_y = rospy.get_param("~last_known_y", 0.0)
        self.last_known_z = rospy.get_param("~last_known_z", 0.0)

        # boundary 支持 rect 和 polygon。rect 可通过 xmin/ymin/xmax/ymax 输入；polygon 可通过 JSON 字符串输入。
        self.task_boundary_type = str(rospy.get_param("~task_boundary_type", "rect")).lower()
        self.task_boundary_xmin = rospy.get_param("~task_boundary_xmin", self.origin_x)
        self.task_boundary_ymin = rospy.get_param("~task_boundary_ymin", self.origin_y)
        self.task_boundary_xmax = rospy.get_param("~task_boundary_xmax", self.origin_x + self.map_size)
        self.task_boundary_ymax = rospy.get_param("~task_boundary_ymax", self.origin_y + self.map_size)
        self.task_boundary_polygon_text = str(rospy.get_param("~task_boundary_polygon", ""))
        self.task_boundary_polygon = []
        self.task_boundary_rect = None

        self.task_probability_sigma = rospy.get_param("~task_probability_sigma", 4.0)
        self.task_probability_floor = rospy.get_param("~task_probability_floor", 0.02)
        self.task_probability_outside_value = rospy.get_param("~task_probability_outside_value", 0.0)
        self.task_enforce_boundary = rospy.get_param("~task_enforce_boundary", True)
        self.task_use_as_prior = rospy.get_param("~task_use_as_prior", True)

        # 没有 LoRa/RF 证据时，可用初始概率作为搜索证据，保证任务输入后能生成初始路线。
        self.use_initial_probability_for_search = rospy.get_param("~use_initial_probability_for_search", True)
        self.initial_probability_evidence_weight = rospy.get_param("~initial_probability_evidence_weight", 0.6)
        self.task_auto_refresh_routes = rospy.get_param("~task_auto_refresh_routes", True)
        self.task_clear_rf_on_update = rospy.get_param("~task_clear_rf_on_update", False)

        # 任务区域入口点：当 START 在任务区域外、01 在任务区域内时，
        # 在 START -> 01 线段与任务边界的交点处生成 ENTRY 标记。
        # 该点用于 RViz 航线表达，不改变高分搜索点筛选逻辑。
        self.task_entry_point_enable = rospy.get_param("~task_entry_point_enable", True)
        self.task_entry_point_min_t = rospy.get_param("~task_entry_point_min_t", 1e-4)
        self.task_entry_point_label = rospy.get_param("~task_entry_point_label", "ENTRY")

        self.task_active = False
        self.task_last_update_time = 0.0
        self.task_update_count = 0
        self.task_probability_map = np.ones((self.height, self.width), dtype=np.float32)

        self.lock = threading.Lock()

        self.rf_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.source_probability_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.source_estimate_x = 0.0
        self.source_estimate_y = 0.0
        self.source_estimate_z = 0.0
        self.source_confidence = 0.0
        self.source_uncertainty_radius = float("inf")
        self.source_sample_count = 0
        self.source_peak_probability = 0.0
        self.source_last_update_time = 0.0
        self.source_dirty = True
        self.source_mode = "INSUFFICIENT_SAMPLES"
        self.prior_map = self.build_prior_map()
        self.fusion_map_2d = np.zeros((self.height, self.width), dtype=np.float32)
        self.terrain_score_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.ground_z_map = np.full((self.height, self.width), np.nan, dtype=np.float32)
        self.slope_map = np.zeros((self.height, self.width), dtype=np.float32)

        # 根据参数初始化搜索任务。若未启用任务，则 task_probability_map 为全 1，算法行为与旧版一致。
        self.initialize_task_from_params()

        # 最近点云缓存：[(stamp, points), ...]
        self.recent_frames = deque()
        self.latest_recent_points = np.empty((0, 3), dtype=np.float32)

        self.obstacle_voxels = set()
        self.wall_columns = set()
        self.free_voxels = set()
        self.has_obstacle_map = False

        self.has_odom = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.last_cloud_time = 0.0
        self.last_score_publish_time = 0.0
        self.last_full_map_rebuild_time = 0.0
        self.full_map_rebuild_requested = True
        self.cached_cloud_frames_since_rebuild = 0
        self.last_full_map_rebuild_stats = {}
        self.lora_count = 0

        self.candidates = self.build_candidates()
        self.neighbor_offsets = self.build_neighbor_offsets()
        self.observed_offsets = self.build_observed_offsets()

        self.raw_lora_pattern = re.compile(
            r"LORA_RX,"
            r"(?:COUNT=(?P<count>\d+),)?"
            r"RSSI=(?P<rssi>-?\d+),"
            r"SNR=(?P<snr>-?\d+\.?\d*),"
            r"SIZE=(?P<size>\d+),"
            r"MSG=(?P<msg>.*)"
        )

        self.pub_rf = rospy.Publisher("/rf_heatmap", OccupancyGrid, queue_size=1)
        self.pub_prior = rospy.Publisher("/prior_heatmap", OccupancyGrid, queue_size=1)
        self.pub_fusion_2d = rospy.Publisher("/fusion_heatmap", OccupancyGrid, queue_size=1)
        self.pub_terrain_score = rospy.Publisher("/terrain_score_map", OccupancyGrid, queue_size=1)
        self.pub_initial_probability = rospy.Publisher(self.initial_probability_topic, OccupancyGrid, queue_size=1)
        self.pub_task_area_markers = rospy.Publisher(self.task_area_markers_topic, MarkerArray, queue_size=1)
        self.pub_search_task_status = rospy.Publisher(self.search_task_status_topic, String, queue_size=10)

        self.pub_recent_cloud = rospy.Publisher(self.recent_cloud_topic, PointCloud2, queue_size=1)
        self.pub_observed_free_cloud = rospy.Publisher(self.observed_free_cloud_topic, PointCloud2, queue_size=1)
        self.pub_score_cloud = rospy.Publisher("/search_3d_score_cloud", PointCloud2, queue_size=1)
        self.pub_goal = rospy.Publisher("/next_search_goal", PoseStamped, queue_size=1)
        self.pub_high_goals = rospy.Publisher(self.high_goals_topic, PoseArray, queue_size=1)
        self.pub_high_goals_cloud = rospy.Publisher(self.high_goals_cloud_topic, PointCloud2, queue_size=1)
        self.pub_waypoints = rospy.Publisher(self.waypoints_topic, PoseArray, queue_size=1)
        self.pub_waypoint_path = rospy.Publisher(self.waypoint_path_topic, Path, queue_size=1)
        self.pub_ordered_goal_array = rospy.Publisher(self.ordered_goal_array_topic, PoseArray, queue_size=1)
        self.pub_ordered_goal_path = rospy.Publisher(self.ordered_goal_path_topic, Path, queue_size=1)
        self.pub_ordered_goal_markers = rospy.Publisher(self.ordered_goal_markers_topic, MarkerArray, queue_size=1)
        self.pub_ordered_goal_sequence = rospy.Publisher(self.ordered_goal_sequence_topic, String, queue_size=10)
        self.pub_backup_goal_markers = rospy.Publisher(self.backup_goal_markers_topic, MarkerArray, queue_size=1)
        self.pub_backup_goal_sequence = rospy.Publisher(self.backup_goal_sequence_topic, String, queue_size=10)
        self.pub_debug = rospy.Publisher("/fusion_debug", String, queue_size=10)
        self.pub_rf_gradient_status = rospy.Publisher("/rf_gradient_status", String, queue_size=10)
        self.pub_rf_gradient_marker = rospy.Publisher("/rf_gradient_marker", MarkerArray, queue_size=1)
        self.pub_source_probability = rospy.Publisher(self.source_probability_topic, OccupancyGrid, queue_size=1)
        self.pub_source_estimate_marker = rospy.Publisher(self.source_estimate_marker_topic, MarkerArray, queue_size=1)
        self.pub_source_estimate_status = rospy.Publisher(self.source_estimate_status_topic, String, queue_size=10)
        self.pub_route_send_status = rospy.Publisher(self.route_send_status_topic, String, queue_size=10)

        self.pub_startup_calib_status = rospy.Publisher(self.startup_calib_status_topic, String, queue_size=10)
        self.pub_startup_calib_result = rospy.Publisher(self.startup_calib_result_topic, String, queue_size=10)
        self.pub_startup_calib_path = rospy.Publisher(self.startup_calib_path_topic, Path, queue_size=1)
        self.pub_startup_calib_markers = rospy.Publisher(self.startup_calib_marker_topic, MarkerArray, queue_size=1)
        self.pub_startup_calib_indicator = rospy.Publisher(self.startup_calib_indicator_topic, String, queue_size=10)

        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        rospy.Subscriber(self.lora_topic, String, self.lora_callback, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(self.ordered_goal_reached_topic, Bool, self.ordered_goal_reached_callback, queue_size=5)
        rospy.Subscriber(self.ordered_goal_reset_topic, Bool, self.ordered_goal_reset_callback, queue_size=5)
        rospy.Subscriber(self.select_backup_route_topic, String, self.select_backup_route_callback, queue_size=5)
        rospy.Subscriber(self.main_routes_refresh_topic, Bool, self.main_routes_refresh_callback, queue_size=5)
        rospy.Subscriber(self.backup_routes_refresh_topic, Bool, self.backup_routes_refresh_callback, queue_size=5)
        rospy.Subscriber(self.routes_refresh_topic, Bool, self.routes_refresh_callback, queue_size=5)
        rospy.Subscriber(self.search_task_topic, String, self.search_task_callback, queue_size=5)
        rospy.Subscriber(self.clear_search_task_topic, Bool, self.clear_search_task_callback, queue_size=5)
        rospy.Subscriber(self.route_send_trigger_topic, Bool, self.route_send_trigger_callback, queue_size=5)
        rospy.Subscriber(self.source_local_route_trigger_topic, Bool, self.source_local_route_trigger_callback, queue_size=5)
        rospy.Subscriber(self.route_send_mission_cmd_topic, String, self.route_send_mission_cmd_callback, queue_size=5)
        rospy.Subscriber(self.target_event_topic, String, self.target_event_callback, queue_size=10)
        rospy.Subscriber(self.startup_calib_start_topic, Bool, self.startup_calib_start_callback, queue_size=5)
        rospy.Subscriber(self.startup_calib_reset_topic, Bool, self.startup_calib_reset_callback, queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        rospy.loginfo("UAV 3D search fusion node started.")
        rospy.loginfo("Cloud topic: %s", self.cloud_topic)
        rospy.loginfo(
            "Recent cloud accumulation: adaptive=%s target=%d max_points=%d voxel=%.3f max_window=%.2f sec legacy_window=%.2f sec",
            str(self.recent_adaptive_accumulation),
            int(self.recent_target_points),
            int(self.max_recent_points),
            float(self.recent_voxel_size),
            float(self.recent_max_window_sec),
            float(self.recent_window_sec)
        )
        rospy.loginfo(
            "Full map rebuild mode: interval_only=%s rebuild_interval=%.2fs score_interval=%.2fs refresh_routes=%s",
            str(self.rebuild_map_only_on_interval),
            float(self.full_map_rebuild_interval),
            float(self.score_publish_interval),
            str(self.full_map_rebuild_refresh_routes)
        )
        rospy.loginfo(
            "Range weighted recent: enable=%s dist=[%.1f, %.1f, %.1f] quota=[%.2f, %.2f, %.2f, %.2f] age=[%.1f, %.1f, %.1f, %.1f] voxel=[%.3f, %.3f, %.3f, %.3f]",
            str(self.enable_range_weighted_recent),
            float(self.range_near_dist),
            float(self.range_mid_dist),
            float(self.range_far_dist),
            float(self.range_near_quota_ratio),
            float(self.range_mid_quota_ratio),
            float(self.range_far_quota_ratio),
            float(self.range_very_far_quota_ratio),
            float(self.range_near_max_age),
            float(self.range_mid_max_age),
            float(self.range_far_max_age),
            float(self.range_very_far_max_age),
            float(self.range_near_voxel),
            float(self.range_mid_voxel),
            float(self.range_far_voxel),
            float(self.range_very_far_voxel)
        )
        rospy.loginfo("Recent obstacle cloud: %s", self.recent_cloud_topic)
        rospy.loginfo("Observed free cloud: %s", self.observed_free_cloud_topic)
        rospy.loginfo("Terrain score map: /terrain_score_map")
        rospy.loginfo("Score cloud: /search_3d_score_cloud")
        rospy.loginfo("High score goals: %s", self.high_goals_topic)
        rospy.loginfo("High score goals cloud: %s", self.high_goals_cloud_topic)
        rospy.loginfo("Search waypoints: %s", self.waypoints_topic)
        rospy.loginfo("Search waypoint path: %s", self.waypoint_path_topic)
        rospy.loginfo("Ordered goal array: %s", self.ordered_goal_array_topic)
        rospy.loginfo("Ordered goal path: %s", self.ordered_goal_path_topic)
        rospy.loginfo("Ordered goal markers: %s", self.ordered_goal_markers_topic)
        rospy.loginfo("Ordered goal sequence: %s", self.ordered_goal_sequence_topic)
        rospy.loginfo("Backup goal markers: %s", self.backup_goal_markers_topic)
        rospy.loginfo("Backup goal sequence: %s", self.backup_goal_sequence_topic)
        rospy.loginfo("Select backup route topic: %s", self.select_backup_route_topic)
        rospy.loginfo("Manual route refresh: %s", str(self.routes_manual_refresh))
        rospy.loginfo("Main route refresh topic: %s", self.main_routes_refresh_topic)
        rospy.loginfo("Backup route refresh topic: %s", self.backup_routes_refresh_topic)
        rospy.loginfo("Combined routes refresh topic: %s", self.routes_refresh_topic)
        rospy.loginfo("Backup routes manual refresh: %s", str(self.backup_routes_manual_refresh))
        rospy.loginfo("Search task input topic: %s", self.search_task_topic)
        rospy.loginfo("Clear search task topic: %s", self.clear_search_task_topic)
        rospy.loginfo("Task area markers: %s", self.task_area_markers_topic)
        rospy.loginfo("Initial probability map: %s", self.initial_probability_topic)
        rospy.loginfo("Search task status: %s", self.search_task_status_topic)
        rospy.loginfo(
            "Search task: active=%s target_id=%s boundary_type=%s enforce_boundary=%s sigma=%.2f use_as_prior=%s initial_as_search=%s",
            str(self.task_active),
            str(self.target_id),
            str(self.task_boundary_type),
            str(self.task_enforce_boundary),
            float(self.task_probability_sigma),
            str(self.task_use_as_prior),
            str(self.use_initial_probability_for_search)
        )
        rospy.loginfo("Ordered goal reached topic: %s", self.ordered_goal_reached_topic)
        rospy.loginfo("Ordered goal reset topic: %s", self.ordered_goal_reset_topic)
        rospy.loginfo("TCP route sender: enable=%s target=%s:%d trigger=%s mission_cmd=%s status=%s",
                      str(self.enable_tcp_route_sender),
                      str(self.jetson_mission_ip),
                      int(self.jetson_mission_port),
                      str(self.route_send_trigger_topic),
                      str(self.route_send_mission_cmd_topic),
                      str(self.route_send_status_topic))
        rospy.loginfo(
            "TCP route return path: append=%s return_to_current=%s route_types=%s",
            str(self.route_send_append_return_path),
            str(self.route_send_return_to_current_position),
            str(self.route_send_return_route_types)
        )
        rospy.loginfo(
            "Target event bridge: enable=%s topic=%s confirm_values=%s cmd=%s",
            str(self.enable_target_event_mission_bridge),
            str(self.target_event_topic),
            str(self.target_event_confirm_values),
            str(self.target_confirm_cmd)
        )
        rospy.loginfo("Ordered goal strict lock: %s", str(self.ordered_goal_strict_lock))
        rospy.loginfo("Ordered goal include current in path/markers: %s", str(self.ordered_goal_include_current_position))
        rospy.loginfo("Ordered goal array include current: %s", str(self.ordered_goal_array_include_current_position))
        rospy.loginfo("Task entry point: enable=%s label=%s", str(self.task_entry_point_enable), str(self.task_entry_point_label))
        rospy.loginfo(
            "Branch routes: enable=%s max_routes=%d max_len=%d attach_radius=%.2f angle_min=%.1f score_ratio=%.2f include_main_prefix=%s",
            str(self.branch_enable),
            int(self.branch_max_routes),
            int(self.branch_max_len),
            float(self.branch_attach_radius),
            float(self.branch_min_angle_deg),
            float(self.branch_score_ratio),
            str(self.branch_include_main_prefix)
        )
        rospy.loginfo(
            "Backup route attach guard: require_attached=%s max_first_dist=%.2f force_attach_seq=%d draw_attach_sphere=%s",
            str(self.branch_require_attached),
            float(self.branch_max_first_branch_dist),
            int(self.branch_force_attach_seq),
            str(self.branch_draw_attach_sphere)
        )
        rospy.loginfo("Candidates: %d", self.candidates.shape[0])
        rospy.loginfo(
            "Region-only task mode: %s draw_last_known_marker=%s terrain_filter=%s search_z=[%.1f, %.1f]",
            str(self.task_region_only_mode),
            str(self.task_draw_last_known_marker),
            str(self.enable_terrain_filter),
            float(self.search_z_min),
            float(self.search_z_max)
        )
        rospy.loginfo(
            "RF gradient: enable=%s publish_continuous=%s window=%.1f min_samples=%d conf_th=%.2f weight=%.2f",
            str(self.enable_rf_gradient),
            str(self.publish_continuous_rf_gradient),
            float(self.rf_gradient_window_sec),
            int(self.rf_gradient_min_samples),
            float(self.rf_gradient_conf_threshold),
            float(self.rf_gradient_weight)
        )
        rospy.loginfo(
            "Source localization: enable=%s topic=%s min_samples=%d window=%.1fs affect_score=%s local_trigger=%s",
            str(self.enable_source_localization),
            str(self.source_probability_topic),
            int(self.source_min_samples),
            float(self.source_window_sec),
            str(self.source_affect_score),
            str(self.source_local_route_trigger_topic)
        )
        rospy.loginfo(
            "Startup RF calibration: enable=%s start_topic=%s marker=%s path=%s route=[%.1f/%.1f, %.1f/%.1f] z=%.1f auto_select=%s",
            str(self.enable_startup_rf_calibration),
            str(self.startup_calib_start_topic),
            str(self.startup_calib_marker_topic),
            str(self.startup_calib_path_topic),
            float(self.startup_calib_forward1),
            float(self.startup_calib_forward2),
            float(self.startup_calib_lateral1),
            float(self.startup_calib_lateral2),
            float(self.startup_calib_z),
            str(self.startup_calib_auto_select_center)
        )
        rospy.loginfo(
            "Startup RF calibration pattern: pattern=%s probe_radius=%.2f return_center=%s sector_count=%d sector_span=%.1f sector_angles=%s",
            str(self.startup_calib_pattern),
            float(self.startup_calib_probe_radius),
            str(self.startup_calib_return_center_between_probes),
            int(self.startup_calib_sector_count),
            float(self.startup_calib_sector_span_deg),
            str(self.startup_calib_sector_angles_deg)
        )
        rospy.loginfo(
            "Startup RF calib anchor: center_lock_to_initial_odom=%s axis_lock_to_initial_yaw=%s use_initial_odom_z=%s",
            str(self.startup_calib_center_lock_to_initial_odom),
            str(self.startup_calib_axis_lock_to_initial_yaw),
            str(self.startup_calib_center_use_initial_odom_z)
        )
        rospy.loginfo(
            "Startup RF calib indicator: enable=%s topic=%s required=%.1fs radius=%.2f check_z=%s require_all_to_finish=%s",
            str(self.startup_calib_indicator_enable),
            str(self.startup_calib_indicator_topic),
            float(self.startup_calib_indicator_required_sec),
            float(self.startup_calib_indicator_radius),
            str(self.startup_calib_indicator_check_z),
            str(self.startup_calib_require_all_points_ready_to_finish)
        )
        rospy.loginfo(
            "Startup RF INIT gate: required_lora_labels=%s min_lora_per_point=%d require_for_init=%s require_to_finish=%s assignment_radius=%.2f",
            str(self.startup_calib_required_lora_labels),
            int(self.startup_calib_min_lora_per_point),
            str(self.startup_calib_require_lora_per_point_for_init),
            str(self.startup_calib_require_lora_per_point_to_finish),
            float(self.startup_calib_sample_assignment_radius)
        )
        rospy.loginfo(
            "Startup RF sample buffer: max_samples=%d keep_all_active=%s",
            int(self.startup_calib_max_samples),
            str(self.startup_calib_keep_all_active_samples)
        )
        rospy.loginfo(
            "LoRa virtual attenuation: atten=%.1f dB rssi_score_range=[%.1f, %.1f] snr_score_max=%.1f rf_gain=%.2f",
            float(self.lora_virtual_attenuation_db),
            float(self.lora_rssi_score_min),
            float(self.lora_rssi_score_max),
            float(self.lora_snr_score_max),
            float(self.rf_gain)
        )

    def route_send_publish_status(self, event, extra=None):
        """发布 DK->Jetson TCP 发送状态，便于 RViz/终端调试。"""
        if extra is None:
            extra = {}

        msg = {
            "event": str(event),
            "stamp": round(float(time.time()), 3),
            "enabled": bool(self.enable_tcp_route_sender),
            "target": "%s:%d" % (str(self.jetson_mission_ip), int(self.jetson_mission_port)),
            "trigger_topic": self.route_send_trigger_topic,
            "mission_cmd_topic": self.route_send_mission_cmd_topic,
            "route_type": self.route_send_route_type,
            "frame_id": self.route_send_frame_id,
            "extra": extra,
        }
        self.pub_route_send_status.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def route_send_sanitize_z(self, z):
        """保证发给飞机的 z 在安全范围内。"""
        try:
            z = float(z)
        except Exception:
            return float(self.route_send_default_z)

        if bool(self.route_send_force_z):
            return float(self.route_send_default_z)

        if (not math.isfinite(z)) or z < float(self.route_send_min_valid_z) or z > float(self.route_send_max_valid_z):
            return float(self.route_send_default_z)

        return float(z)

    def route_send_estimate_yaw(self, points, idx):
        """根据相邻航点估计 yaw。points: [(x,y,z,score,label,kind), ...]。"""
        if not points:
            return 0.0

        try:
            x, y = float(points[idx][0]), float(points[idx][1])

            if idx + 1 < len(points):
                nx, ny = float(points[idx + 1][0]), float(points[idx + 1][1])
                return math.atan2(ny - y, nx - x)

            if idx > 0:
                px, py = float(points[idx - 1][0]), float(points[idx - 1][1])
                return math.atan2(y - py, x - px)

            if self.has_odom:
                return float(self.current_yaw)
        except Exception:
            pass

        return 0.0

    def route_send_route_type_allows_return(self, route_type):
        allowed = set()
        for item in str(self.route_send_return_route_types).split(","):
            item = item.strip().upper()
            if item:
                allowed.add(item)
        return str(route_type).upper() in allowed

    def route_send_append_return_points(self, route_points, route_type):
        """
        给发送到无人机的航点追加原路返回段。
        输入/输出格式均为 [(x,y,z,score,label,kind), ...]。
        """
        points = list(route_points)
        if not points:
            return points

        if not bool(self.route_send_append_return_path):
            return points

        if not self.route_send_route_type_allows_return(route_type):
            return points

        return_points = []

        # 不重复最后一个点；从倒数第二个点开始沿原路线回退。
        for x, y, z, score, label, _kind in reversed(points[:-1]):
            return_points.append((
                float(x),
                float(y),
                float(z),
                float(score),
                "R-%s" % str(label),
                "return",
            ))

        # 追加路线下发瞬间的无人机当前位置，形成真正“回到出发附近”。
        if bool(self.route_send_return_to_current_position) and self.has_odom:
            return_points.append((
                float(self.current_x),
                float(self.current_y),
                self.route_send_sanitize_z(float(self.current_z)),
                0.0,
                "START",
                "return_start",
            ))

        if return_points:
            rospy.loginfo(
                "Append return path: route_type=%s forward=%d return=%d total=%d",
                str(route_type),
                len(points),
                len(return_points),
                len(points) + len(return_points)
            )

        return points + return_points

    def build_tcp_route_packet_from_ordered_goals(self, ordered_goals, route_id=None, route_type=None):
        """
        将当前稳定主路线打包成 Jetson Mission Supervisor 可直接接收的 JSON。
        注意：默认只发送真正搜索航点，不发送 START；ENTRY 可通过 route_send_include_entry 开启。
        """
        if not ordered_goals:
            return None

        route_type = str(route_type or self.route_send_route_type or "MAIN_ROUTE")
        max_goals = max(1, int(self.route_send_max_goals))

        route_points = []
        if bool(self.route_send_include_entry):
            entry = self.compute_task_entry_point_to_first_goal(ordered_goals)
            if entry is not None:
                ex, ey, ez, _escore, elabel, ekind = entry
                route_points.append((float(ex), float(ey), float(ez), 0.0, str(elabel), str(ekind)))

        for i, (x, y, z, score) in enumerate(ordered_goals, start=1):
            route_points.append((float(x), float(y), float(z), float(score), "%02d" % i, "goal"))

        route_points = route_points[:max_goals]
        route_points = self.route_send_append_return_points(route_points, route_type)

        goals = []
        for i, (x, y, z, score, label, kind) in enumerate(route_points, start=1):
            z = self.route_send_sanitize_z(z)
            yaw = self.route_send_estimate_yaw(route_points, i - 1)
            hold_sec = float(self.route_send_return_hold_sec) if str(kind).startswith("return") else float(self.route_send_hold_sec)
            goals.append({
                "seq": int(i),
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "z": round(float(z), 4),
                "yaw": round(float(yaw), 4),
                "hold": hold_sec,
                "name": "dk_%s_%02d" % (str(kind), int(i)),
                "source": "dk2500_fusion_tcp",
                "score": round(float(score), 4),
                "label": str(label),
                "kind": str(kind),
            })

        if not goals:
            return None

        if route_id is None:
            route_id = "dk_tcp_%s_%d" % (route_type.lower(), int(time.time()))

        packet = {
            "cmd": "ROUTE_UPDATE",
            "route_type": route_type,
            "route_id": str(route_id),
            "frame_id": str(self.route_send_frame_id or self.frame_id),
            "preempt": False,
            "goals": goals,
            "source": "dk2500_uav_3d_search_fusion",
            "meta": {
                "queue_action": str(self.stable_ordered_goal_last_action),
                "queue_value": round(float(self.stable_ordered_goal_value), 4),
                "threshold": round(float(self.stable_ordered_goal_threshold), 4),
                "max_score": round(float(self.stable_ordered_goal_max_score), 4),
                "include_entry": bool(self.route_send_include_entry),
                "append_return_path": bool(self.route_send_append_return_path),
                "return_to_current_position": bool(self.route_send_return_to_current_position),
                "task_active": bool(self.task_active),
                "target_id": str(self.target_id),
            }
        }
        return packet

    def tcp_send_json_packet(self, packet, event_prefix="route"):
        """一行一个 JSON，以 \n 结尾，发送到 Jetson Mission Supervisor TCP Server。"""
        if not bool(self.enable_tcp_route_sender):
            self.route_send_publish_status("tcp_sender_disabled", {"packet_cmd": packet.get("cmd", "") if isinstance(packet, dict) else ""})
            rospy.logwarn("TCP route sender is disabled.")
            return False

        if not isinstance(packet, dict):
            self.route_send_publish_status("tcp_send_failed", {"reason": "packet_not_dict"})
            return False

        try:
            line = json.dumps(packet, ensure_ascii=False, separators=(",", ":")) + "\n"
            sock = socket.create_connection(
                (str(self.jetson_mission_ip), int(self.jetson_mission_port)),
                timeout=float(self.route_send_connect_timeout)
            )
            sock.settimeout(float(self.route_send_send_timeout))
            sock.sendall(line.encode("utf-8"))
            sock.close()

            extra = {
                "cmd": packet.get("cmd", ""),
                "route_id": packet.get("route_id", ""),
                "route_type": packet.get("route_type", ""),
                "goal_count": len(packet.get("goals", [])) if isinstance(packet.get("goals", []), list) else 0,
                "bytes": len(line.encode("utf-8")),
            }
            self.route_send_publish_status("tcp_send_ok", extra)
            rospy.loginfo("TCP send OK to %s:%d cmd=%s route_id=%s goals=%d",
                          str(self.jetson_mission_ip),
                          int(self.jetson_mission_port),
                          str(extra["cmd"]),
                          str(extra["route_id"]),
                          int(extra["goal_count"]))
            return True

        except Exception as e:
            extra = {
                "cmd": packet.get("cmd", ""),
                "route_id": packet.get("route_id", ""),
                "error": str(e),
            }
            self.route_send_publish_status("tcp_send_failed", extra)
            rospy.logerr("TCP send failed to %s:%d: %s", str(self.jetson_mission_ip), int(self.jetson_mission_port), str(e))
            return False

    def route_send_trigger_callback(self, msg):
        """
        RViz Panel 的 Send Route 按钮发布 std_msgs/Bool True 到 /send_route_to_uav。
        本回调把当前稳定主路线 stable_ordered_goals 发送到 Jetson Mission Supervisor。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            ordered_goals = list(self.stable_ordered_goals)
            queue_action = str(self.stable_ordered_goal_last_action)
            queue_value = float(self.stable_ordered_goal_value)

        if not ordered_goals:
            self.route_send_publish_status("no_route_to_send", {
                "reason": "stable_ordered_goals_empty",
                "queue_action": queue_action,
                "queue_value": round(queue_value, 4),
            })
            rospy.logwarn("No stable ordered goals to send. Refresh main route first.")
            return

        packet = self.build_tcp_route_packet_from_ordered_goals(ordered_goals)
        if packet is None:
            self.route_send_publish_status("build_route_packet_failed", {"reason": "empty_packet"})
            return

        self.tcp_send_json_packet(packet, event_prefix="route")

    def route_send_mission_cmd_callback(self, msg):
        """
        DK 端向 /dk_tcp_mission_cmd 发布 std_msgs/String JSON，转发给 Jetson。
        示例：{"cmd":"RETURN_HOME","reason":"dk_panel"}
        """
        text = str(msg.data).strip()
        if not text:
            return

        try:
            packet = json.loads(text)
        except Exception as e:
            self.route_send_publish_status("bad_mission_cmd_json", {"error": str(e), "text": text[:200]})
            rospy.logwarn("Bad mission cmd JSON: %s", str(e))
            return

        if not isinstance(packet, dict):
            self.route_send_publish_status("bad_mission_cmd_not_dict", {"text": text[:200]})
            return

        self.tcp_send_json_packet(packet, event_prefix="mission_cmd")

    def target_event_values_set(self, text):
        values = set()
        for item in str(text).split(","):
            item = item.strip().upper()
            if item:
                values.add(item)
        return values

    def target_event_callback(self, msg):
        """
        接入 OpenVINO/YOLO 识别事件。
        收到 TARGET_CONFIRM 后，向 Jetson uav_mission_supervisor 发送 TARGET_CONFIRM，
        使无人机进入目标确认悬停状态。
        """
        if not bool(self.enable_target_event_mission_bridge):
            return

        text = str(msg.data).strip()
        if not text:
            return

        try:
            event_pkt = json.loads(text)
        except Exception as e:
            self.route_send_publish_status("bad_target_event_json", {"error": str(e), "text": text[:200]})
            rospy.logwarn("Bad target event JSON: %s", str(e))
            return

        if not isinstance(event_pkt, dict):
            self.route_send_publish_status("bad_target_event_not_dict", {"text": text[:200]})
            return

        event = str(event_pkt.get("event", "")).upper().strip()
        self.last_target_event = event

        confirm_values = self.target_event_values_set(self.target_event_confirm_values)
        lost_values = self.target_event_values_set(self.target_event_lost_values)

        if event in lost_values:
            rospy.loginfo_throttle(2.0, "Target event bridge: target lost/no target event=%s", event)
            return

        if event not in confirm_values:
            rospy.loginfo_throttle(2.0, "Target event bridge: ignore event=%s", event)
            return

        now = time.time()
        min_interval = max(0.0, float(self.target_confirm_min_interval))
        if self.last_target_confirm_send_time > 0.0 and now - self.last_target_confirm_send_time < min_interval:
            rospy.loginfo_throttle(
                2.0,
                "Target event bridge: TARGET_CONFIRM suppressed by interval %.1fs",
                min_interval
            )
            return

        packet = {
            "cmd": str(self.target_confirm_cmd),
            "reason": "openvino_person_confirmed",
            "source": "dk2500_target_event_bridge",
            "preempt": bool(self.target_confirm_preempt),
            "stamp": round(float(now), 3),
            "target_event": event_pkt,
        }

        ok = self.tcp_send_json_packet(packet, event_prefix="target_confirm")
        if ok:
            self.last_target_confirm_send_time = now
            self.route_send_publish_status("target_confirm_hover_sent", {
                "event": event,
                "conf": event_pkt.get("conf", event_pkt.get("conf_avg", None)),
                "target_id": event_pkt.get("target_id", ""),
                "cmd": str(self.target_confirm_cmd),
            })
            rospy.logwarn(
                "TARGET_CONFIRM received from %s, sent hover/confirm command to UAV. conf=%s",
                str(self.target_event_topic),
                str(event_pkt.get("conf", event_pkt.get("conf_avg", "")))
            )

    def initialize_task_from_params(self):
        """
        从 ROS 参数初始化搜索任务。
        task_enabled=False 时保持旧行为：prior_map 按 use_prior 生成，task_probability_map 为全 1。
        """
        self.task_boundary_polygon = self.parse_task_polygon_text(self.task_boundary_polygon_text)

        if self.task_region_only_mode:
            self.task_has_last_known = False

        if not bool(self.task_enabled):
            self.task_active = False
            self.task_probability_map = np.zeros((self.height, self.width), dtype=np.float32)
            return

        self.normalize_task_boundary()
        self.task_active = True
        self.task_last_update_time = time.time()
        self.task_update_count += 1
        self.task_probability_map = self.build_task_probability_map()

        if self.task_use_as_prior:
            self.prior_map = np.copy(self.task_probability_map)

    def parse_task_polygon_text(self, text):
        text = str(text).strip()
        if not text:
            return []

        try:
            data = json.loads(text)
            pts = []
            for item in data:
                if isinstance(item, dict):
                    pts.append((float(item["x"]), float(item["y"])))
                else:
                    pts.append((float(item[0]), float(item[1])))
            return pts if len(pts) >= 3 else []
        except Exception as e:
            rospy.logwarn("Parse task boundary polygon failed: %s", str(e))
            return []

    def normalize_task_boundary(self):
        btype = str(self.task_boundary_type).lower()

        if btype == "polygon" and len(self.task_boundary_polygon) >= 3:
            self.task_boundary_type = "polygon"
            self.task_boundary_rect = None
            return

        # 默认使用矩形边界。即使用户传入 xmax < xmin，也自动纠正。
        xmin = min(float(self.task_boundary_xmin), float(self.task_boundary_xmax))
        xmax = max(float(self.task_boundary_xmin), float(self.task_boundary_xmax))
        ymin = min(float(self.task_boundary_ymin), float(self.task_boundary_ymax))
        ymax = max(float(self.task_boundary_ymin), float(self.task_boundary_ymax))

        # 限制到全局 map 范围内，避免 OccupancyGrid 越界。
        map_xmin = float(self.origin_x)
        map_xmax = float(self.origin_x + self.map_size)
        map_ymin = float(self.origin_y)
        map_ymax = float(self.origin_y + self.map_size)

        xmin = max(map_xmin, min(map_xmax, xmin))
        xmax = max(map_xmin, min(map_xmax, xmax))
        ymin = max(map_ymin, min(map_ymax, ymin))
        ymax = max(map_ymin, min(map_ymax, ymax))

        if xmax - xmin < self.resolution:
            xmax = min(map_xmax, xmin + self.resolution)
        if ymax - ymin < self.resolution:
            ymax = min(map_ymax, ymin + self.resolution)

        self.task_boundary_type = "rect"
        self.task_boundary_xmin = xmin
        self.task_boundary_xmax = xmax
        self.task_boundary_ymin = ymin
        self.task_boundary_ymax = ymax
        self.task_boundary_rect = (xmin, ymin, xmax, ymax)

    def point_in_polygon_xy(self, x, y, polygon):
        inside = False
        n = len(polygon)
        if n < 3:
            return False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)):
                x_cross = (xj - xi) * (y - yi) / max((yj - yi), 1e-12) + xi
                if x < x_cross:
                    inside = not inside
            j = i

        return inside

    def point_in_task_boundary(self, x, y):
        if not self.task_active:
            return True

        if not self.task_enforce_boundary:
            return True

        btype = str(self.task_boundary_type).lower()
        if btype == "polygon" and len(self.task_boundary_polygon) >= 3:
            return self.point_in_polygon_xy(float(x), float(y), self.task_boundary_polygon)

        if self.task_boundary_rect is None:
            self.normalize_task_boundary()

        xmin, ymin, xmax, ymax = self.task_boundary_rect
        return (xmin <= float(x) <= xmax) and (ymin <= float(y) <= ymax)

    def build_task_probability_map(self):
        prob = np.zeros((self.height, self.width), dtype=np.float32)
        sigma = max(float(self.task_probability_sigma), self.resolution)
        floor_value = float(np.clip(self.task_probability_floor, 0.0, 1.0))
        outside_value = float(np.clip(self.task_probability_outside_value, 0.0, 1.0))

        for gy in range(self.height):
            for gx in range(self.width):
                wx, wy = self.grid_to_world(gx, gy)
                inside = self.point_in_task_boundary(wx, wy)

                if not inside:
                    prob[gy, gx] = outside_value
                    continue

                if (not self.task_region_only_mode) and self.task_has_last_known:
                    d2 = (wx - float(self.last_known_x)) ** 2 + (wy - float(self.last_known_y)) ** 2
                    p = math.exp(-d2 / (2.0 * sigma * sigma))
                    prob[gy, gx] = max(floor_value, p)
                else:
                    prob[gy, gx] = 1.0

        max_p = float(np.max(prob)) if prob.size > 0 else 0.0
        if max_p > 1e-6:
            prob = prob / max_p

        return prob.astype(np.float32)

    def refresh_prior_from_task_locked(self):
        if self.task_active and self.task_use_as_prior:
            self.prior_map = np.copy(self.task_probability_map)
        else:
            self.prior_map = self.build_prior_map()

        self.fusion_map_2d = self.rf_map * self.prior_map

    def request_route_refresh_after_task_update_locked(self):
        if not self.task_auto_refresh_routes:
            return

        with self.queue_lock:
            self.ordered_goal_reset_requested = True
            self.backup_routes_refresh_requested = True
            self.cached_backup_valid = False
            self.cached_backup_action = "task_update_refresh_requested"
            self.selected_backup_route_id = None

    def set_search_task(self, target_id=None, last_known=None, boundary=None, sigma=None,
                        probability_floor=None, enforce_boundary=None):
        """
        更新搜索任务。
        boundary 支持：
        1. {"type":"rect", "xmin":..., "ymin":..., "xmax":..., "ymax":...}
        2. {"type":"polygon", "points":[[x1,y1], [x2,y2], ...]}
        """
        if target_id is not None:
            self.target_id = str(target_id)

        if self.task_region_only_mode:
            last_known = None
            self.task_has_last_known = False

        if last_known is not None:
            if isinstance(last_known, dict):
                self.last_known_x = float(last_known.get("x", self.last_known_x))
                self.last_known_y = float(last_known.get("y", self.last_known_y))
                self.last_known_z = float(last_known.get("z", self.last_known_z))
            else:
                self.last_known_x = float(last_known[0])
                self.last_known_y = float(last_known[1])
                if len(last_known) >= 3:
                    self.last_known_z = float(last_known[2])
            self.task_has_last_known = True

        if boundary is not None:
            if isinstance(boundary, dict):
                btype = str(boundary.get("type", "rect")).lower()
                self.task_boundary_type = btype
                if btype == "polygon":
                    pts = boundary.get("points", boundary.get("polygon", []))
                    self.task_boundary_polygon = []
                    for item in pts:
                        if isinstance(item, dict):
                            self.task_boundary_polygon.append((float(item["x"]), float(item["y"])))
                        else:
                            self.task_boundary_polygon.append((float(item[0]), float(item[1])))
                else:
                    self.task_boundary_xmin = float(boundary.get("xmin", boundary.get("x_min", self.task_boundary_xmin)))
                    self.task_boundary_ymin = float(boundary.get("ymin", boundary.get("y_min", self.task_boundary_ymin)))
                    self.task_boundary_xmax = float(boundary.get("xmax", boundary.get("x_max", self.task_boundary_xmax)))
                    self.task_boundary_ymax = float(boundary.get("ymax", boundary.get("y_max", self.task_boundary_ymax)))
                    self.task_boundary_polygon = []
                    self.task_boundary_type = "rect"
            elif isinstance(boundary, (list, tuple)) and len(boundary) >= 4:
                self.task_boundary_type = "rect"
                self.task_boundary_xmin = float(boundary[0])
                self.task_boundary_ymin = float(boundary[1])
                self.task_boundary_xmax = float(boundary[2])
                self.task_boundary_ymax = float(boundary[3])
                self.task_boundary_polygon = []

        if sigma is not None:
            self.task_probability_sigma = float(sigma)

        if probability_floor is not None:
            self.task_probability_floor = float(probability_floor)

        if enforce_boundary is not None:
            self.task_enforce_boundary = bool(enforce_boundary)

        if self.task_region_only_mode:
            self.task_has_last_known = False

        self.normalize_task_boundary()
        self.task_active = True
        self.task_enabled = True
        self.task_last_update_time = time.time()
        self.task_update_count += 1
        self.task_probability_map = self.build_task_probability_map()

        if self.task_clear_rf_on_update:
            self.rf_map[:, :] = 0.0

        self.source_dirty = True
        self.refresh_prior_from_task_locked()
        self.request_route_refresh_after_task_update_locked()

    def search_task_callback(self, msg):
        """
        运行时设置搜索任务。
        示例：
        {"target_id":"T001", "last_known":{"x":0,"y":0,"z":0},
         "boundary":{"type":"rect","xmin":-6,"ymin":-4,"xmax":6,"ymax":5}, "sigma":3.0}
        """
        try:
            data = json.loads(str(msg.data))
        except Exception as e:
            rospy.logwarn("Set search task failed: invalid JSON: %s", str(e))
            return

        target_id = data.get("target_id", data.get("id", self.target_id))

        last_known = data.get("last_known", None)
        if last_known is None and ("last_known_x" in data or "x" in data):
            last_known = {
                "x": data.get("last_known_x", data.get("x", self.last_known_x)),
                "y": data.get("last_known_y", data.get("y", self.last_known_y)),
                "z": data.get("last_known_z", data.get("z", self.last_known_z))
            }

        boundary = data.get("boundary", None)
        if boundary is None and any(k in data for k in ["xmin", "x_min", "boundary_xmin"]):
            boundary = {
                "type": "rect",
                "xmin": data.get("xmin", data.get("x_min", data.get("boundary_xmin", self.task_boundary_xmin))),
                "ymin": data.get("ymin", data.get("y_min", data.get("boundary_ymin", self.task_boundary_ymin))),
                "xmax": data.get("xmax", data.get("x_max", data.get("boundary_xmax", self.task_boundary_xmax))),
                "ymax": data.get("ymax", data.get("y_max", data.get("boundary_ymax", self.task_boundary_ymax)))
            }

        sigma = data.get("sigma", data.get("probability_sigma", data.get("task_probability_sigma", None)))
        floor_value = data.get("floor", data.get("probability_floor", None))
        enforce_boundary = data.get("enforce_boundary", None)

        with self.lock:
            self.set_search_task(
                target_id=target_id,
                last_known=last_known,
                boundary=boundary,
                sigma=sigma,
                probability_floor=floor_value,
                enforce_boundary=enforce_boundary
            )

        rospy.loginfo(
            "Search task updated: target_id=%s last_known=(%.2f, %.2f, %.2f) boundary=%s sigma=%.2f",
            str(self.target_id),
            float(self.last_known_x),
            float(self.last_known_y),
            float(self.last_known_z),
            str(self.task_boundary_type),
            float(self.task_probability_sigma)
        )

    def clear_search_task_callback(self, msg):
        if not bool(msg.data):
            return

        with self.lock:
            self.task_active = False
            self.task_enabled = False
            self.task_probability_map = np.zeros((self.height, self.width), dtype=np.float32)
            self.source_probability_map[:, :] = 0.0
            self.source_confidence = 0.0
            self.source_uncertainty_radius = float("inf")
            self.source_mode = "TASK_CLEARED"
            self.refresh_prior_from_task_locked()
            self.request_route_refresh_after_task_update_locked()

        rospy.loginfo("Search task cleared. Task boundary and initial probability are disabled.")

    def get_task_probability_score(self, x, y):
        if not self.task_active:
            return 1.0

        idx = self.world_to_grid(x, y)
        if idx is None:
            return 0.0

        gx, gy = idx
        return float(self.task_probability_map[gy, gx])

    def segment_intersection_xy(self, ax, ay, bx, by, cx, cy, dx, dy):
        """
        计算二维线段 AB 与 CD 的交点参数。
        返回 (t, u)，其中：
        A + t * (B - A) = C + u * (D - C)
        t/u 均在 [0,1] 内表示两条线段相交。
        平行或不相交返回 None。
        """
        rx = float(bx) - float(ax)
        ry = float(by) - float(ay)
        sx = float(dx) - float(cx)
        sy = float(dy) - float(cy)
        denom = rx * sy - ry * sx

        if abs(denom) < 1e-9:
            return None

        qpx = float(cx) - float(ax)
        qpy = float(cy) - float(ay)
        t = (qpx * sy - qpy * sx) / denom
        u = (qpx * ry - qpy * rx) / denom

        eps = 1e-6
        if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
            return float(max(0.0, min(1.0, t))), float(max(0.0, min(1.0, u)))

        return None

    def compute_task_entry_point_to_first_goal(self, ordered_goals):
        """
        当无人机当前位置 START 在任务区域外、首个搜索点在任务区域内时，
        计算 START -> 01 与任务边界的交点，并作为 ENTRY 入口点返回。

        返回格式：
            (x, y, z, score, label, kind)
        其中 kind='entry'。
        """
        if not bool(self.task_entry_point_enable):
            return None

        if not self.task_active or not self.has_odom or not ordered_goals:
            return None

        sx = float(self.current_x)
        sy = float(self.current_y)
        sz = float(self.current_z)

        # START 已经在任务区域内时，不需要额外入口点。
        if self.point_in_task_boundary(sx, sy):
            return None

        gx, gy, gz, _score = ordered_goals[0]
        gx = float(gx)
        gy = float(gy)
        gz = float(gz)

        boundary_pts = self.task_boundary_points()
        if len(boundary_pts) < 2:
            return None

        best_t = None
        for i in range(len(boundary_pts) - 1):
            x1, y1 = boundary_pts[i]
            x2, y2 = boundary_pts[i + 1]
            hit = self.segment_intersection_xy(sx, sy, gx, gy, x1, y1, x2, y2)
            if hit is None:
                continue

            t, _u = hit
            if t < float(self.task_entry_point_min_t):
                continue

            if best_t is None or t < best_t:
                best_t = t

        if best_t is None:
            return None

        ex = sx + best_t * (gx - sx)
        ey = sy + best_t * (gy - sy)
        ez = sz + best_t * (gz - sz)

        return (float(ex), float(ey), float(ez), 0.0, str(self.task_entry_point_label), "entry")

    def build_ordered_visual_route_points(self, ordered_goals, include_current=None):
        """
        构造用于 RViz 显示的完整任务航线点序列。

        与 /ordered_goal_array 不同，这个序列强调可视化表达：
            START -> ENTRY -> 01 -> 02 -> ...
        其中 ENTRY 只在 START 位于任务区域外且能与任务边界求交时出现。
        """
        if include_current is None:
            include_current = self.ordered_goal_include_current_position

        route_points = []

        if include_current and self.has_odom:
            route_points.append((
                float(self.current_x),
                float(self.current_y),
                float(self.current_z),
                0.0,
                "START",
                "start"
            ))

            entry = self.compute_task_entry_point_to_first_goal(ordered_goals)
            if entry is not None:
                route_points.append(entry)

        for i, (x, y, z, score) in enumerate(ordered_goals, start=1):
            route_points.append((float(x), float(y), float(z), float(score), "%02d" % i, "goal"))

        return route_points


    def task_boundary_points(self):
        if not self.task_active:
            return []

        if str(self.task_boundary_type).lower() == "polygon" and len(self.task_boundary_polygon) >= 3:
            pts = list(self.task_boundary_polygon)
            if pts[0] != pts[-1]:
                pts.append(pts[0])
            return pts

        if self.task_boundary_rect is None:
            self.normalize_task_boundary()

        xmin, ymin, xmax, ymax = self.task_boundary_rect
        return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax), (xmin, ymin)]

    def publish_task_area_markers(self):
        markers = MarkerArray()
        stamp = rospy.Time.now()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not self.task_active:
            self.pub_task_area_markers.publish(markers)
            return

        lifetime = rospy.Duration(0.0)
        pts = self.task_boundary_points()

        # 矩形任务区域半透明填充。
        if str(self.task_boundary_type).lower() == "rect" and self.task_boundary_rect is not None:
            xmin, ymin, xmax, ymax = self.task_boundary_rect
            fill = Marker()
            fill.header.stamp = stamp
            fill.header.frame_id = self.frame_id
            fill.ns = "task_area_fill"
            fill.id = 0
            fill.type = Marker.CUBE
            fill.action = Marker.ADD
            fill.pose.position.x = 0.5 * (xmin + xmax)
            fill.pose.position.y = 0.5 * (ymin + ymax)
            fill.pose.position.z = 0.02
            fill.pose.orientation.w = 1.0
            fill.scale.x = max(xmax - xmin, self.resolution)
            fill.scale.y = max(ymax - ymin, self.resolution)
            fill.scale.z = 0.02
            fill.color.r = 1.0
            fill.color.g = 0.8
            fill.color.b = 0.0
            fill.color.a = 0.13
            fill.lifetime = lifetime
            markers.markers.append(fill)

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.frame_id
        line.ns = "task_area_boundary"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.10
        line.color.r = 1.0
        line.color.g = 0.9
        line.color.b = 0.0
        line.color.a = 1.0
        line.lifetime = lifetime

        for x, y in pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.12
            line.points.append(p)

        markers.markers.append(line)

        if (not self.task_region_only_mode) and self.task_draw_last_known_marker and self.task_has_last_known:
            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self.frame_id
            sphere.ns = "task_last_known"
            sphere.id = 0
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(self.last_known_x)
            sphere.pose.position.y = float(self.last_known_y)
            sphere.pose.position.z = float(self.last_known_z) + 0.25
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.45
            sphere.scale.y = 0.45
            sphere.scale.z = 0.45
            sphere.color.r = 1.0
            sphere.color.g = 0.2
            sphere.color.b = 0.1
            sphere.color.a = 1.0
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self.frame_id
            text.ns = "task_last_known_text"
            text.id = 0
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(self.last_known_x)
            text.pose.position.y = float(self.last_known_y)
            text.pose.position.z = float(self.last_known_z) + 0.95
            text.pose.orientation.w = 1.0
            text.scale.z = 0.55
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            label = "Target %s\\nLast known" % (self.target_id if self.target_id else "UNKNOWN")
            text.text = label
            text.lifetime = lifetime
            markers.markers.append(text)

        self.pub_task_area_markers.publish(markers)

    def make_search_task_status_json(self):
        if not self.task_active:
            data = {
                "event": "search_task_status",
                "active": False,
                "task_region_only_mode": bool(self.task_region_only_mode),
                "search_region_is_last_seen_area": bool(self.task_region_only_mode),
                "set_topic": self.search_task_topic,
                "initial_probability_topic": self.initial_probability_topic,
                "task_area_markers_topic": self.task_area_markers_topic
            }
            return json.dumps(data, ensure_ascii=False)

        boundary = {"type": self.task_boundary_type}
        if self.task_boundary_type == "polygon":
            boundary["points"] = [[round(float(x), 3), round(float(y), 3)] for x, y in self.task_boundary_polygon]
        else:
            xmin, ymin, xmax, ymax = self.task_boundary_rect
            boundary.update({
                "xmin": round(float(xmin), 3),
                "ymin": round(float(ymin), 3),
                "xmax": round(float(xmax), 3),
                "ymax": round(float(ymax), 3)
            })

        data = {
            "event": "search_task_status",
            "active": True,
            "target_id": self.target_id,
            "task_region_only_mode": bool(self.task_region_only_mode),
            "search_region_is_last_seen_area": bool(self.task_region_only_mode),
            "last_known": {
                "available": bool((not self.task_region_only_mode) and self.task_has_last_known),
                "x": round(float(self.last_known_x), 3),
                "y": round(float(self.last_known_y), 3),
                "z": round(float(self.last_known_z), 3)
            },
            "boundary": boundary,
            "sigma": round(float(self.task_probability_sigma), 3),
            "floor": round(float(self.task_probability_floor), 4),
            "enforce_boundary": bool(self.task_enforce_boundary),
            "use_as_prior": bool(self.task_use_as_prior),
            "initial_as_search_evidence": bool(self.use_initial_probability_for_search),
            "initial_probability_topic": self.initial_probability_topic,
            "task_area_markers_topic": self.task_area_markers_topic,
            "set_topic": self.search_task_topic,
            "clear_topic": self.clear_search_task_topic,
            "auto_refresh_routes": bool(self.task_auto_refresh_routes),
            "update_count": int(self.task_update_count),
            "age": round(float(0.0 if self.task_last_update_time <= 0.0 else max(0.0, time.time() - self.task_last_update_time)), 2)
        }
        return json.dumps(data, ensure_ascii=False)

    def publish_search_task_status(self):
        self.pub_search_task_status.publish(String(data=self.make_search_task_status_json()))

    def build_candidates(self):
        xs = np.arange(self.origin_x, self.origin_x + self.map_size, self.candidate_step_xy)
        ys = np.arange(self.origin_y, self.origin_y + self.map_size, self.candidate_step_xy)
        zs = np.arange(self.search_z_min, self.search_z_max + 1e-6, self.candidate_step_z)

        candidates = []
        for z in zs:
            for y in ys:
                for x in xs:
                    candidates.append([x, y, z])

        return np.asarray(candidates, dtype=np.float32)

    def build_neighbor_offsets(self):
        r = int(math.ceil(self.safe_radius / self.obstacle_voxel_size))
        offsets = []

        for ix in range(-r, r + 1):
            for iy in range(-r, r + 1):
                for iz in range(-r, r + 1):
                    d = math.sqrt(ix * ix + iy * iy + iz * iz) * self.obstacle_voxel_size
                    if d <= self.safe_radius:
                        offsets.append((d, ix, iy, iz))

        offsets.sort(key=lambda x: x[0])
        return offsets

    def build_observed_offsets(self):
        r = int(math.ceil(self.observed_neighbor_radius / self.obstacle_voxel_size))
        offsets = []

        for ix in range(-r, r + 1):
            for iy in range(-r, r + 1):
                for iz in range(-r, r + 1):
                    d = math.sqrt(ix * ix + iy * iy + iz * iz) * self.obstacle_voxel_size
                    if d <= self.observed_neighbor_radius:
                        offsets.append((ix, iy, iz))

        return offsets

    def build_prior_map(self):
        prior = np.ones((self.height, self.width), dtype=np.float32)

        if not self.use_prior:
            return prior

        for gy in range(self.height):
            for gx in range(self.width):
                wx, wy = self.grid_to_world(gx, gy)
                d2 = (wx - self.prior_x) ** 2 + (wy - self.prior_y) ** 2
                prior[gy, gx] = math.exp(-d2 / (2.0 * self.prior_sigma ** 2))

        prior = prior / max(float(np.max(prior)), 1e-6)
        return prior.astype(np.float32)

    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = self.normalize_angle(
            self.quat_to_yaw(msg.pose.pose.orientation) + self.yaw_offset
        )
        self.has_odom = True

    def ordered_goal_reached_callback(self, msg):
        """
        外部控制器确认已经到达当前队首 goal 后，发布 std_msgs/Bool True 到 /ordered_goal_reached。
        收到该信号后只删除队首一个 goal，剩余队列顺序不变。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            self.ordered_goal_external_reached_requested = True

        rospy.loginfo("Received external ordered goal reached signal.")

    def ordered_goal_reset_callback(self, msg):
        """
        发布 std_msgs/Bool True 到 /ordered_goal_reset 后，只刷新主线路。
        备用线路缓存保持不变；需要重建备用线路时单独发布 /backup_routes_refresh。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            self.ordered_goal_reset_requested = True
            self.selected_backup_route_id = None

        rospy.loginfo("Received ordered goal reset signal: main route refresh requested; backup route cache kept.")

    def main_routes_refresh_callback(self, msg):
        """
        只刷新主线路。
        示例：rostopic pub /main_routes_refresh std_msgs/Bool "data: true" -1
        收到后：清空稳定主队列；下一周期用当前高分点重建主线路。备用线路不变。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            self.ordered_goal_reset_requested = True
            self.selected_backup_route_id = None

        rospy.loginfo("Received main route refresh signal. Backup route cache is kept.")

    def backup_routes_refresh_callback(self, msg):
        """
        只刷新备用线路。
        示例：rostopic pub /backup_routes_refresh std_msgs/Bool "data: true" -1
        收到后：保持当前稳定主线路不变，基于当前主线路重新生成备用分叉线路。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            self.backup_routes_refresh_requested = True
            self.cached_backup_valid = False
            self.cached_backup_action = "manual_backup_refresh_requested"
            self.selected_backup_route_id = None

        rospy.loginfo("Received backup route refresh signal. Main route queue is kept.")

    def routes_refresh_callback(self, msg):
        """
        兼容保留：同时刷新主线路与备用线路。
        示例：rostopic pub /search_routes_refresh std_msgs/Bool "data: true" -1
        收到后：清空稳定主队列；下一周期用当前高分点重建主线路，并同步重建备用分叉线路。
        """
        if not bool(msg.data):
            return

        with self.queue_lock:
            self.ordered_goal_reset_requested = True
            self.backup_routes_refresh_requested = True
            self.cached_backup_valid = False
            self.cached_backup_action = "manual_all_routes_refresh_requested"
            self.selected_backup_route_id = None

        rospy.loginfo("Received combined refresh signal for main and backup routes.")

    def select_backup_route_callback(self, msg):
        """
        手动选择备用线路。
        示例：rostopic pub /select_backup_route std_msgs/String "data: 'B1'" -1
        下一次路径发布周期会把主队列替换为：分叉点 + B1 备用线路。
        """
        route_id = str(msg.data).strip().upper()
        if not route_id:
            return

        with self.queue_lock:
            self.selected_backup_route_id = route_id

        rospy.loginfo("Received backup route selection: %s", route_id)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)

        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return None

        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def point_to_voxel(self, x, y, z):
        return (
            int(math.floor(x / self.obstacle_voxel_size)),
            int(math.floor(y / self.obstacle_voxel_size)),
            int(math.floor(z / self.obstacle_voxel_size)),
        )

    def point_to_xy_voxel(self, x, y):
        return (
            int(math.floor(x / self.obstacle_voxel_size)),
            int(math.floor(y / self.obstacle_voxel_size)),
        )

    def voxel_to_point(self, key):
        vx, vy, vz = key
        s = self.obstacle_voxel_size
        return [
            (vx + 0.5) * s,
            (vy + 0.5) * s,
            (vz + 0.5) * s
        ]

    def point_range_from_uav(self, p):
        """
        点到当前无人机位置的距离。
        有 odom 时使用 UAV 当前位置；没有 odom 时退化为点到 map 原点的距离。
        """
        if self.has_odom:
            dx = float(p[0]) - float(self.current_x)
            dy = float(p[1]) - float(self.current_y)
            dz = float(p[2]) - float(self.current_z)
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        return math.sqrt(float(p[0]) ** 2 + float(p[1]) ** 2 + float(p[2]) ** 2)

    def recent_range_bin(self, p):
        """
        0: near, 1: mid, 2: far, 3: very_far
        """
        d = self.point_range_from_uav(p)
        if d <= float(self.range_near_dist):
            return 0
        if d <= float(self.range_mid_dist):
            return 1
        if d <= float(self.range_far_dist):
            return 2
        return 3

    def recent_range_name(self, bin_id):
        names = ["near", "mid", "far", "very_far"]
        return names[int(np.clip(bin_id, 0, 3))]

    def recent_range_max_age(self, bin_id):
        ages = [
            float(self.range_near_max_age),
            float(self.range_mid_max_age),
            float(self.range_far_max_age),
            float(self.range_very_far_max_age),
        ]
        return max(0.0, ages[int(np.clip(bin_id, 0, 3))])

    def recent_range_voxel_size(self, bin_id):
        if not self.enable_range_weighted_recent:
            return max(float(self.recent_voxel_size), 1e-4)

        sizes = [
            float(self.range_near_voxel),
            float(self.range_mid_voxel),
            float(self.range_far_voxel),
            float(self.range_very_far_voxel),
        ]
        return max(sizes[int(np.clip(bin_id, 0, 3))], 1e-4)

    def recent_range_quota_counts(self, target_points=None):
        if target_points is None:
            target_points = int(self.recent_target_points)

        target_points = max(1, int(target_points))
        ratios = np.asarray([
            float(self.range_near_quota_ratio),
            float(self.range_mid_quota_ratio),
            float(self.range_far_quota_ratio),
            float(self.range_very_far_quota_ratio),
        ], dtype=np.float32)

        ratios = np.maximum(ratios, 0.0)
        s = float(np.sum(ratios))
        if s <= 1e-6:
            ratios = np.asarray([0.15, 0.25, 0.35, 0.25], dtype=np.float32)
            s = float(np.sum(ratios))

        ratios = ratios / s
        quotas = np.floor(ratios * target_points).astype(np.int32)

        # 保证总和正好等于 target_points。
        diff = target_points - int(np.sum(quotas))
        for i in range(abs(diff)):
            idx = i % 4
            quotas[idx] += 1 if diff > 0 else -1

        quotas = np.maximum(quotas, 0)
        return [int(x) for x in quotas]

    def format_recent_range_counts(self):
        c = self.last_recent_range_counts
        return "near=%d mid=%d far=%d very_far=%d" % (int(c[0]), int(c[1]), int(c[2]), int(c[3]))

    def recent_voxel_key(self, p):
        if self.enable_range_weighted_recent:
            bin_id = self.recent_range_bin(p)
            s = self.recent_range_voxel_size(bin_id)
            return (
                int(bin_id),
                int(math.floor(float(p[0]) / s)),
                int(math.floor(float(p[1]) / s)),
                int(math.floor(float(p[2]) / s)),
            )

        s = max(float(self.recent_voxel_size), 1e-4)
        return (
            int(math.floor(float(p[0]) / s)),
            int(math.floor(float(p[1]) / s)),
            int(math.floor(float(p[2]) / s)),
        )

    def voxel_downsample_recent(self, points):
        if points.shape[0] == 0:
            return points

        voxel_dict = {}
        for p in points:
            voxel_dict[self.recent_voxel_key(p)] = p

        return np.asarray(list(voxel_dict.values()), dtype=np.float32)

    def recent_active_window_sec(self):
        if self.recent_adaptive_accumulation:
            return max(float(self.recent_max_window_sec), float(self.recent_window_sec))
        return float(self.recent_window_sec)

    def remove_old_recent_frames(self, now):
        """
        删除超过最大允许时间窗的旧帧。
        注意：adaptive 模式下不会因为超过 recent_window_sec 就立刻删帧；
        它会保留到 recent_max_window_sec，以便 /uav_recent_obstacle_cloud 能累积到目标点数。
        """
        window_sec = self.recent_active_window_sec()

        while self.recent_frames:
            stamp, _ = self.recent_frames[0]
            if now - stamp <= window_sec:
                break
            self.recent_frames.popleft()

    def downsample_points_by_voxel_cap(self, points, max_points):
        """
        总点数超限时按距离层限额抽样。
        这样近处密集点不会把 max_recent_points 全部占满，远处点会被优先保留。
        """
        if points.shape[0] == 0:
            return points

        max_points = int(max_points)
        if points.shape[0] <= max_points:
            return points.astype(np.float32)

        if not self.enable_range_weighted_recent:
            idx = np.random.choice(points.shape[0], max_points, replace=False)
            return points[idx].astype(np.float32)

        quotas = self.recent_range_quota_counts(max_points)
        bin_indices = [[] for _ in range(4)]
        for i, p in enumerate(points):
            b = self.recent_range_bin(p)
            bin_indices[b].append(i)

        selected = []
        for b in [0, 1, 2, 3]:
            idx = np.asarray(bin_indices[b], dtype=np.int64)
            if idx.size == 0 or quotas[b] <= 0:
                continue
            if idx.size <= quotas[b]:
                selected.extend(idx.tolist())
            else:
                chosen = np.random.choice(idx, quotas[b], replace=False)
                selected.extend(chosen.tolist())

        # 如果远处点不足导致没有达到 max_points，补点顺序优先 far/very_far/mid，near 最后。
        if len(selected) < max_points:
            selected_set = set(selected)
            fill_order = [2, 3, 1, 0]
            for b in fill_order:
                remaining = [i for i in bin_indices[b] if i not in selected_set]
                if not remaining:
                    continue
                if b == 0 and not self.range_allow_near_overflow:
                    continue
                need = max_points - len(selected)
                if need <= 0:
                    break
                remaining = np.asarray(remaining, dtype=np.int64)
                add_n = min(need, remaining.size)
                add = np.random.choice(remaining, add_n, replace=False)
                selected.extend(add.tolist())
                selected_set.update(add.tolist())

        if not selected:
            idx = np.random.choice(points.shape[0], max_points, replace=False)
            return points[idx].astype(np.float32)

        selected = np.asarray(selected[:max_points], dtype=np.int64)
        return points[selected].astype(np.float32)

    def build_recent_points_fixed_window_locked(self):
        """
        旧逻辑：固定时间窗内所有帧合并，然后体素下采样。
        仅在 ~recent_adaptive_accumulation:=false 时使用。
        """
        if not self.recent_frames:
            self.last_recent_raw_points_seen = 0
            self.last_recent_frames_used = 0
            self.last_recent_oldest_age = 0.0
            self.last_recent_downsampled_points = 0
            return np.empty((0, 3), dtype=np.float32)

        clouds = [pts for _, pts in self.recent_frames]
        if not clouds:
            self.last_recent_raw_points_seen = 0
            self.last_recent_frames_used = 0
            self.last_recent_oldest_age = 0.0
            self.last_recent_downsampled_points = 0
            return np.empty((0, 3), dtype=np.float32)

        points = np.vstack(clouds)
        self.last_recent_raw_points_seen = int(points.shape[0])
        self.last_recent_frames_used = int(len(clouds))

        now = time.time()
        oldest_stamp = self.recent_frames[0][0]
        self.last_recent_oldest_age = max(0.0, now - oldest_stamp)

        points = self.voxel_downsample_recent(points)
        points = self.downsample_points_by_voxel_cap(points, int(self.max_recent_points))
        self.last_recent_downsampled_points = int(points.shape[0])
        return points

    def build_recent_points_adaptive_locked(self, now):
        """
        距离分层滚动累计。

        普通累计会让近处密集点长期占据缓存，远处点仍然稀疏。
        这里把点按距离分为 near/mid/far/very_far 四层：
        - near：累计时间短、点数预算少、体素较大；
        - far/very_far：累计时间长、点数预算多、体素较小。

        这样 /uav_recent_obstacle_cloud 不会伪造远处点，但会把缓存资源更多分配给远处真实观测点。
        """
        if not self.recent_frames:
            self.last_recent_raw_points_seen = 0
            self.last_recent_frames_used = 0
            self.last_recent_oldest_age = 0.0
            self.last_recent_downsampled_points = 0
            self.last_recent_range_counts = [0, 0, 0, 0]
            self.last_recent_range_raw_seen = [0, 0, 0, 0]
            return np.empty((0, 3), dtype=np.float32)

        if not self.enable_range_weighted_recent:
            target_points = max(1, int(self.recent_target_points))
            max_points = max(target_points, int(self.max_recent_points))
            max_window_sec = max(float(self.recent_max_window_sec), float(self.recent_window_sec))
            min_window_sec = max(0.0, float(self.recent_min_window_sec))

            voxel_dict = {}
            frames_used = 0
            raw_points_seen = 0
            oldest_used_stamp = now

            for stamp, pts in reversed(self.recent_frames):
                age = now - stamp
                if age > max_window_sec:
                    continue

                frames_used += 1
                raw_points_seen += int(pts.shape[0])
                oldest_used_stamp = stamp

                for p in pts:
                    key = self.recent_voxel_key(p)
                    if key not in voxel_dict:
                        voxel_dict[key] = p

                if len(voxel_dict) >= target_points and age >= min_window_sec:
                    break

                if len(voxel_dict) >= max_points:
                    break

            if not voxel_dict:
                self.last_recent_raw_points_seen = 0
                self.last_recent_frames_used = 0
                self.last_recent_oldest_age = 0.0
                self.last_recent_downsampled_points = 0
                self.last_recent_range_counts = [0, 0, 0, 0]
                self.last_recent_range_raw_seen = [0, 0, 0, 0]
                return np.empty((0, 3), dtype=np.float32)

            points = np.asarray(list(voxel_dict.values()), dtype=np.float32)
            points = self.downsample_points_by_voxel_cap(points, int(self.max_recent_points))

            counts = [0, 0, 0, 0]
            for p in points:
                counts[self.recent_range_bin(p)] += 1

            self.last_recent_raw_points_seen = int(raw_points_seen)
            self.last_recent_frames_used = int(frames_used)
            self.last_recent_oldest_age = max(0.0, now - oldest_used_stamp)
            self.last_recent_downsampled_points = int(points.shape[0])
            self.last_recent_range_counts = counts
            self.last_recent_range_raw_seen = [0, 0, 0, 0]
            return points

        target_points = max(1, int(self.recent_target_points))
        max_points = max(target_points, int(self.max_recent_points))
        global_max_window_sec = max(float(self.recent_max_window_sec), float(self.recent_window_sec))
        min_window_sec = max(0.0, float(self.recent_min_window_sec))

        quotas = self.recent_range_quota_counts(target_points)
        voxel_dicts = [dict(), dict(), dict(), dict()]
        raw_seen_bins = [0, 0, 0, 0]
        frames_used = 0
        raw_points_seen = 0
        oldest_used_stamp = now

        # 从最新帧向旧帧回溯。每层独立累计：近处很快达到 quota 并停止增加，远处继续向旧帧累计。
        for stamp, pts in reversed(self.recent_frames):
            age = now - stamp
            if age > global_max_window_sec:
                continue

            frames_used += 1
            raw_points_seen += int(pts.shape[0])
            oldest_used_stamp = stamp

            for p in pts:
                b = self.recent_range_bin(p)
                raw_seen_bins[b] += 1

                # 近处只保留最近几秒；远处可以保留更久。
                if age > self.recent_range_max_age(b):
                    continue

                # 近处达到预算后不再继续累积，避免近处白成一片。
                if len(voxel_dicts[b]) >= quotas[b]:
                    continue

                key = self.recent_voxel_key(p)
                if key not in voxel_dicts[b]:
                    voxel_dicts[b][key] = p

            total_kept = sum(len(d) for d in voxel_dicts)
            all_quota_full = all(len(voxel_dicts[i]) >= quotas[i] for i in range(4))

            # 目标达到后可停止；如果某些层不足，则继续向历史帧找远处点，直到窗口上限。
            if total_kept >= target_points and age >= min_window_sec:
                break

            if all_quota_full:
                break

            if total_kept >= max_points:
                break

        # 合并输出。默认不让 near 用溢出点补满总数；这样近处不会重新占满缓存。
        points_by_bin = []
        for b in range(4):
            if voxel_dicts[b]:
                points_by_bin.append(np.asarray(list(voxel_dicts[b].values()), dtype=np.float32))
            else:
                points_by_bin.append(np.empty((0, 3), dtype=np.float32))

        if not any(p.shape[0] > 0 for p in points_by_bin):
            self.last_recent_raw_points_seen = 0
            self.last_recent_frames_used = 0
            self.last_recent_oldest_age = 0.0
            self.last_recent_downsampled_points = 0
            self.last_recent_range_counts = [0, 0, 0, 0]
            self.last_recent_range_raw_seen = raw_seen_bins
            return np.empty((0, 3), dtype=np.float32)

        points = np.vstack([p for p in points_by_bin if p.shape[0] > 0])
        points = self.downsample_points_by_voxel_cap(points, int(self.max_recent_points))

        counts = [0, 0, 0, 0]
        for p in points:
            counts[self.recent_range_bin(p)] += 1

        self.last_recent_raw_points_seen = int(raw_points_seen)
        self.last_recent_frames_used = int(frames_used)
        self.last_recent_oldest_age = max(0.0, now - oldest_used_stamp)
        self.last_recent_downsampled_points = int(points.shape[0])
        self.last_recent_range_counts = counts
        self.last_recent_range_raw_seen = raw_seen_bins

        return points

    def build_recent_points_locked(self):
        if self.recent_adaptive_accumulation:
            return self.build_recent_points_adaptive_locked(time.time())
        return self.build_recent_points_fixed_window_locked()

    def points_to_cloud_msg(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id

        if points.shape[0] == 0:
            return pc2.create_cloud_xyz32(header, [])

        return pc2.create_cloud_xyz32(header, points.tolist())

    def publish_recent_obstacle_cloud(self, points):
        msg = self.points_to_cloud_msg(points)
        self.pub_recent_cloud.publish(msg)

    def publish_observed_free_cloud(self, free_voxels):
        if not free_voxels:
            points = np.empty((0, 3), dtype=np.float32)
        else:
            points = np.asarray(
                [self.voxel_to_point(k) for k in free_voxels],
                dtype=np.float32
            )

        msg = self.points_to_cloud_msg(points)
        self.pub_observed_free_cloud.publish(msg)

    def build_free_voxels_from_rays(self, obstacle_points):
        free_voxels = set()

        if not self.has_odom:
            return free_voxels

        # /uav_recent_obstacle_cloud 可以达到 5 万点左右，但 observed free 射线不必对所有点做，
        # 否则 DK2500 上 CPU 压力会很高。这里随机抽样一部分点构建 free_voxels。
        max_ray_points = int(max(0, self.max_free_ray_points))
        if max_ray_points > 0 and obstacle_points.shape[0] > max_ray_points:
            idx = np.random.choice(obstacle_points.shape[0], max_ray_points, replace=False)
            ray_points = obstacle_points[idx]
        else:
            ray_points = obstacle_points

        sx = self.current_x
        sy = self.current_y
        sz = self.current_z

        for x, y, z in ray_points:
            dx = x - sx
            dy = y - sy
            dz = z - sz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < 1e-6:
                continue

            usable_dist = max(0.0, dist - self.free_ray_stop_before)
            steps = int(usable_dist / self.free_ray_step)

            for i in range(1, steps + 1):
                t = (i * self.free_ray_step) / dist

                if t <= 0.0 or t >= 1.0:
                    continue

                px = sx + dx * t
                py = sy + dy * t
                pz = sz + dz * t

                if self.world_to_grid(px, py) is None:
                    continue

                if pz < self.search_z_min or pz > self.search_z_max:
                    continue

                free_voxels.add(self.point_to_voxel(px, py, pz))

        return free_voxels

    def inflate_wall_columns(self, wall_columns):
        inflated = set()
        r = int(math.ceil(self.wall_inflation_radius / self.obstacle_voxel_size))

        for vx, vy in wall_columns:
            for ix in range(-r, r + 1):
                for iy in range(-r, r + 1):
                    d = math.sqrt(ix * ix + iy * iy) * self.obstacle_voxel_size
                    if d <= self.wall_inflation_radius:
                        inflated.add((vx + ix, vy + iy))

        return inflated

    def rebuild_terrain_maps_locked(self, points):
        z_values = [[[] for _ in range(self.width)] for _ in range(self.height)]

        for x, y, z in points:
            idx = self.world_to_grid(x, y)
            if idx is None:
                continue

            gx, gy = idx
            z_values[gy][gx].append(float(z))

        ground = np.full((self.height, self.width), np.nan, dtype=np.float32)
        height_span = np.zeros((self.height, self.width), dtype=np.float32)
        valid = np.zeros((self.height, self.width), dtype=np.bool_)

        for gy in range(self.height):
            for gx in range(self.width):
                zs = z_values[gy][gx]

                if len(zs) < self.min_terrain_points:
                    continue

                zs = np.asarray(zs, dtype=np.float32)

                # 用低分位数估计地面，避免少量噪点直接影响 ground_z
                ground_z = float(np.percentile(zs, 10))
                top_z = float(np.percentile(zs, 90))

                ground[gy, gx] = ground_z
                height_span[gy, gx] = max(0.0, top_z - ground_z)
                valid[gy, gx] = True

        slope = np.zeros((self.height, self.width), dtype=np.float32)
        terrain_score = np.zeros((self.height, self.width), dtype=np.float32)

        for gy in range(1, self.height - 1):
            for gx in range(1, self.width - 1):
                if not valid[gy, gx]:
                    continue

                # 室内点云稀疏，不能要求上下左右四邻域全部有效。
                # 有成对邻居时估计坡度；缺邻居时认为坡度信息未知，不直接惩罚。
                has_dx = valid[gy, gx - 1] and valid[gy, gx + 1]
                has_dy = valid[gy - 1, gx] and valid[gy + 1, gx]

                dzdx = 0.0
                dzdy = 0.0

                if has_dx:
                    dzdx = (ground[gy, gx + 1] - ground[gy, gx - 1]) / (2.0 * self.resolution)

                if has_dy:
                    dzdy = (ground[gy + 1, gx] - ground[gy - 1, gx]) / (2.0 * self.resolution)

                if has_dx or has_dy:
                    slope_deg = math.degrees(math.atan(math.sqrt(dzdx * dzdx + dzdy * dzdy)))
                else:
                    slope_deg = 0.0

                slope[gy, gx] = slope_deg

                if slope_deg <= self.max_walkable_slope_deg:
                    slope_score = 1.0
                elif slope_deg >= self.cliff_slope_deg:
                    slope_score = 0.0
                else:
                    t = (
                        (slope_deg - self.max_walkable_slope_deg) /
                        max(self.cliff_slope_deg - self.max_walkable_slope_deg, 1e-6)
                    )
                    slope_score = 1.0 - t

                if height_span[gy, gx] <= self.max_ground_height_span:
                    roughness_score = 1.0
                else:
                    # 高度起伏大时只软惩罚，避免室内墙/窗帘点云把整片区域压死。
                    over = height_span[gy, gx] - self.max_ground_height_span
                    roughness_score = max(0.5, 1.0 - over / max(self.max_ground_height_span, 1e-6))

                terrain_score[gy, gx] = float(np.clip(slope_score * roughness_score, 0.0, 1.0))

        self.ground_z_map = ground
        self.slope_map = slope
        self.terrain_score_map = terrain_score

    def rebuild_obstacle_map_locked(self):
        now = time.time()
        self.remove_old_recent_frames(now)

        obstacle_points = self.build_recent_points_locked()

        if obstacle_points.shape[0] == 0:
            self.obstacle_voxels = set()
            self.wall_columns = set()
            self.free_voxels = set()
            self.latest_recent_points = np.empty((0, 3), dtype=np.float32)
            self.terrain_score_map = np.zeros((self.height, self.width), dtype=np.float32)
            self.ground_z_map = np.full((self.height, self.width), np.nan, dtype=np.float32)
            self.slope_map = np.zeros((self.height, self.width), dtype=np.float32)
            self.has_obstacle_map = False
            return 0, 0, 0, 0

        if obstacle_points.shape[0] > self.max_obstacle_points:
            idx = np.random.choice(obstacle_points.shape[0], self.max_obstacle_points, replace=False)
            obstacle_points = obstacle_points[idx]

        column_stats = {}
        voxels = set()

        for x, y, z in obstacle_points:
            voxels.add(self.point_to_voxel(x, y, z))

            key_xy = self.point_to_xy_voxel(x, y)
            if key_xy not in column_stats:
                column_stats[key_xy] = [z, z, 1]
            else:
                column_stats[key_xy][0] = min(column_stats[key_xy][0], z)
                column_stats[key_xy][1] = max(column_stats[key_xy][1], z)
                column_stats[key_xy][2] += 1

        raw_wall_columns = set()
        for key_xy, stat in column_stats.items():
            z_min, z_max, count = stat
            z_span = z_max - z_min

            if count >= self.wall_min_points and z_span >= self.wall_min_z_span:
                raw_wall_columns.add(key_xy)

        wall_columns = self.inflate_wall_columns(raw_wall_columns)
        free_voxels = self.build_free_voxels_from_rays(obstacle_points)
        self.rebuild_terrain_maps_locked(obstacle_points)

        self.obstacle_voxels = voxels
        self.wall_columns = wall_columns
        self.free_voxels = free_voxels
        self.latest_recent_points = obstacle_points
        self.has_obstacle_map = True

        return obstacle_points.shape[0], len(voxels), len(wall_columns), len(free_voxels)

    def perform_full_map_rebuild_locked(self, now):
        """
        统一重建全地图：recent obstacle cloud、obstacle voxels、wall columns、
        observed free voxels、terrain score map。

        注意：调用者必须已经持有 self.lock。
        """
        n_points, n_voxels, n_walls, n_free = self.rebuild_obstacle_map_locked()
        recent_points = np.copy(self.latest_recent_points)
        free_voxels = set(self.free_voxels)

        self.last_full_map_rebuild_time = float(now)
        self.full_map_rebuild_requested = False
        cached_before = int(self.cached_cloud_frames_since_rebuild)
        self.cached_cloud_frames_since_rebuild = 0

        stats = {
            "recent_points": int(n_points),
            "voxels": int(n_voxels),
            "wall_columns": int(n_walls),
            "free_voxels": int(n_free),
            "cached_frames": int(len(self.recent_frames)),
            "cached_frames_since_last_rebuild": cached_before,
            "raw_seen": int(self.last_recent_raw_points_seen),
            "frames_used": int(self.last_recent_frames_used),
            "oldest_age": float(self.last_recent_oldest_age),
            "range_counts": self.format_recent_range_counts(),
            "terrain_max": float(np.max(self.terrain_score_map)) if self.terrain_score_map.size > 0 else 0.0,
            "frame_id": str(self.frame_id),
        }
        self.last_full_map_rebuild_stats = stats

        if bool(self.full_map_rebuild_refresh_routes):
            with self.queue_lock:
                self.ordered_goal_reset_requested = True
                self.backup_routes_refresh_requested = True
                self.cached_backup_valid = False
                self.cached_backup_action = "full_map_rebuild_refresh_requested"

        return stats, recent_points, free_voxels

    def cloud_callback(self, msg):
        now = time.time()

        if now - self.last_cloud_time < self.cloud_process_interval:
            return

        self.last_cloud_time = now

        frame_points = []

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p

            if z < self.obstacle_z_min or z > self.obstacle_z_max:
                continue

            if self.world_to_grid(x, y) is None:
                continue

            frame_points.append([x, y, z])

            if len(frame_points) >= self.max_input_points_per_frame:
                break

        if not frame_points:
            rospy.logwarn_throttle(2.0, "No valid points in current cloud frame.")
            return

        frame_points = np.asarray(frame_points, dtype=np.float32)
        frame_points = frame_points[np.isfinite(frame_points).all(axis=1)]

        if frame_points.shape[0] == 0:
            return

        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

        with self.lock:
            self.recent_frames.append((now, frame_points))
            self.remove_old_recent_frames(now)
            self.full_map_rebuild_requested = True
            self.cached_cloud_frames_since_rebuild += 1

            # 兼容旧模式：关闭 interval-only 后，仍然每次 cloud_callback 都重建地图。
            if not bool(self.rebuild_map_only_on_interval):
                stats, recent_points, free_voxels = self.perform_full_map_rebuild_locked(now)
            else:
                stats, recent_points, free_voxels = None, None, None

        if stats is not None:
            self.publish_recent_obstacle_cloud(recent_points)
            self.publish_observed_free_cloud(free_voxels)
            rospy.loginfo_throttle(
                2.0,
                "Recent obstacle map updated: recent_points=%d target=%d raw_seen=%d frames_used=%d cached_frames=%d oldest_age=%.1fs range_counts=[%s] voxels=%d wall_columns=%d free_voxels=%d terrain_max=%.3f frame=%s",
                int(stats["recent_points"]),
                int(self.recent_target_points),
                int(stats["raw_seen"]),
                int(stats["frames_used"]),
                int(stats["cached_frames"]),
                float(stats["oldest_age"]),
                str(stats["range_counts"]),
                int(stats["voxels"]),
                int(stats["wall_columns"]),
                int(stats["free_voxels"]),
                float(stats["terrain_max"]),
                str(stats["frame_id"])
            )
        else:
            rospy.loginfo_throttle(
                5.0,
                "Cloud frame cached only: frame_points=%d cached_frames=%d since_rebuild=%d next_full_rebuild_in=%.1fs frame=%s",
                int(frame_points.shape[0]),
                int(len(self.recent_frames)),
                int(self.cached_cloud_frames_since_rebuild),
                max(0.0, float(self.full_map_rebuild_interval) - (now - float(self.last_full_map_rebuild_time))),
                str(self.frame_id)
            )

    def parse_lora(self, text):
        text = text.strip()

        try:
            data = json.loads(text)
            return {
                "rssi": float(data["rssi"]),
                "snr": float(data["snr"]),
                "msg": str(data.get("msg", "")),
                "x": None if data.get("x", None) is None else float(data["x"]),
                "y": None if data.get("y", None) is None else float(data["y"]),
                "z": None if data.get("z", None) is None else float(data["z"]),
            }
        except Exception:
            pass

        match = self.raw_lora_pattern.search(text)
        if not match:
            return None

        return {
            "rssi": float(match.group("rssi")),
            "snr": float(match.group("snr")),
            "msg": match.group("msg"),
            "x": None,
            "y": None,
            "z": None
        }

    def lora_callback(self, msg):
        data = self.parse_lora(msg.data)

        if data is None:
            rospy.logwarn_throttle(2.0, "LoRa parse failed.")
            return

        if data["snr"] < self.min_snr:
            rospy.logwarn_throttle(2.0, "Ignore LoRa: low SNR %.2f", data["snr"])
            return

        if self.target_prefix and not data["msg"].startswith(self.target_prefix):
            rospy.logwarn_throttle(2.0, "Ignore LoRa: invalid msg %s", data["msg"])
            return

        if data["x"] is not None and data["y"] is not None:
            sx = data["x"]
            sy = data["y"]
            sz = 0.0 if data["z"] is None else data["z"]
        else:
            if not self.has_odom:
                rospy.logwarn_throttle(2.0, "LoRa has no position and odom not ready.")
                return

            sx = self.current_x
            sy = self.current_y
            sz = self.current_z

        idx = self.world_to_grid(sx, sy)
        if idx is None:
            rospy.logwarn("LoRa sample outside map: %.2f %.2f", sx, sy)
            return

        raw_rssi = float(data["rssi"])
        snr = float(data["snr"])

        # 小场地虚拟衰减：只改变算法使用的 RSSI，不改变真实 LoRa 接收。
        attenuation_db = float(self.lora_virtual_attenuation_db)
        rssi = raw_rssi - attenuation_db

        rssi_min = float(self.lora_rssi_score_min)
        rssi_max = float(self.lora_rssi_score_max)
        if rssi_max <= rssi_min:
            rssi_max = rssi_min + 1.0

        rssi_score = np.clip((rssi - rssi_min) / (rssi_max - rssi_min), 0.0, 1.0)
        snr_score = np.clip(snr / max(float(self.lora_snr_score_max), 1e-6), 0.0, 1.0)
        confidence = float(rssi_score * snr_score)

        sigma = 0.8 + (1.0 - rssi_score) * 4.0
        radius = int(max(3, 3.0 * sigma / self.resolution))

        gx0, gy0 = idx

        with self.lock:
            x_min = max(0, gx0 - radius)
            x_max = min(self.width, gx0 + radius + 1)
            y_min = max(0, gy0 - radius)
            y_max = min(self.height, gy0 + radius + 1)

            for gy in range(y_min, y_max):
                for gx in range(x_min, x_max):
                    wx, wy = self.grid_to_world(gx, gy)
                    d = math.hypot(wx - sx, wy - sy)
                    w = math.exp(-(d * d) / (2.0 * sigma * sigma))
                    self.rf_map[gy, gx] += self.rf_gain * confidence * w

            self.rf_map = np.clip(self.rf_map, 0.0, 1.0)

            self.add_rf_sample_locked(
                sx=sx,
                sy=sy,
                sz=sz,
                rssi=rssi,
                raw_rssi=raw_rssi,
                virtual_attenuation_db=attenuation_db,
                snr=snr,
                rssi_score=float(rssi_score),
                snr_score=float(snr_score),
                confidence=float(confidence)
            )
            # 连续 RF 梯度被关闭时，不再用“最近几个点”滚动拟合/刷新梯度。
            # 开局 INIT RF 仍由 startup calibration 结束时一次性计算。
            if bool(self.enable_rf_gradient):
                self.update_rf_gradient_locked()

        self.lora_count += 1

        debug = {
            "event": "lora_fused",
            "raw_rssi": round(float(raw_rssi), 1),
            "effective_rssi": round(float(rssi), 1),
            "rssi": round(float(rssi), 1),
            "virtual_attenuation_db": round(float(attenuation_db), 1),
            "rssi_score": round(float(rssi_score), 3),
            "snr": round(float(snr), 2),
            "snr_score": round(float(snr_score), 3),
            "confidence": round(confidence, 3),
            "x": round(sx, 3),
            "y": round(sy, 3),
            "z": round(sz, 3),
            "yaw": round(self.current_yaw, 3),
            "count": self.lora_count,
            "rf_mode": self.rf_search_mode,
            "rf_quality": round(float(self.rf_latest_quality), 3),
            "rf_trend": round(float(self.rf_gradient_trend), 3),
            "rf_gradient": {
                "x": round(float(self.rf_gradient_x), 4),
                "y": round(float(self.rf_gradient_y), 4),
                "conf": round(float(self.rf_gradient_conf), 3),
                "heading_deg": round(
                    math.degrees(math.atan2(self.rf_gradient_y, self.rf_gradient_x))
                    if math.hypot(self.rf_gradient_x, self.rf_gradient_y) > 1e-6 else 0.0,
                    1
                )
            },
            "source_estimate": {
                "mode": str(self.source_mode),
                "x": round(float(self.source_estimate_x), 3),
                "y": round(float(self.source_estimate_y), 3),
                "confidence": round(float(self.source_confidence), 3),
                "uncertainty_radius": None if not math.isfinite(float(self.source_uncertainty_radius)) else round(float(self.source_uncertainty_radius), 3)
            }
        }

        self.pub_debug.publish(String(data=json.dumps(debug, ensure_ascii=False)))
        if bool(self.publish_continuous_rf_gradient):
            self.publish_rf_gradient_status_and_marker()

        rospy.loginfo(
            "LoRa fused: rawRSSI=%.1f effectiveRSSI=%.1f atten=%.1f SNR=%.2f pos=(%.2f, %.2f, %.2f) conf=%.2f",
            raw_rssi,
            rssi,
            attenuation_db,
            snr,
            sx,
            sy,
            sz,
            confidence
        )


    def add_rf_sample_locked(self, sx, sy, sz, rssi, raw_rssi=None, virtual_attenuation_db=0.0,
                             snr=0.0, rssi_score=0.0, snr_score=0.0, confidence=0.0):
        """
        保存 LoRa 采样历史。

        逻辑修正：
        1) 连续 RF 梯度仍然可以对近距离/短时间重复点做降采样；
        2) 开局校准专用样本 self.startup_calib_samples 必须完整保留 active 期间的有效 LoRa，
           不受连续梯度去重、不受 expected_duration_sec 滚动窗口影响；
        3) 这样 P0/F/B/L/R 每个点采到的 LoRa 不会因为手动校准时间较长而被清掉。
        """
        # 样本服务于三类模块：
        # 1) 连续 RF 梯度；2) 源概率定位；3) 开局 RF 校准。
        # 即使关闭连续 RF 梯度，只要还需要源定位或开局校准，也必须保留 LoRa 样本。
        if (not self.enable_rf_gradient) and (not self.enable_source_localization) and (not self.enable_startup_rf_calibration):
            return

        now = time.time()

        # quality 更适合判断信号强弱趋势；confidence 仍用于 RF 热力图融合。
        quality = 0.75 * float(rssi_score) + 0.25 * float(snr_score)
        quality = float(np.clip(quality, 0.0, 1.0))

        sample = {
            "t": float(now),
            "x": float(sx),
            "y": float(sy),
            "z": float(sz),
            # rssi/effective_rssi 是算法实际使用的 RSSI；raw_rssi 是 LoRa 模块真实上报值。
            "rssi": float(rssi),
            "effective_rssi": float(rssi),
            "raw_rssi": float(rssi if raw_rssi is None else raw_rssi),
            "virtual_attenuation_db": float(virtual_attenuation_db),
            "snr": float(snr),
            "rssi_score": float(rssi_score),
            "snr_score": float(snr_score),
            "confidence": float(confidence),
            "quality": float(quality)
        }

        # 开局校准样本必须优先、完整地单独保存。
        # 这一步放在连续 RF 梯度的重复点降采样之前，避免同一点停留时 LoRa 被丢掉，导致每点 count 不够。
        if bool(self.enable_startup_rf_calibration) and bool(self.startup_calib_active) and float(self.startup_calib_start_time) > 0.0:
            if float(sample["t"]) >= float(self.startup_calib_start_time):
                if not hasattr(self, "startup_calib_samples"):
                    self.startup_calib_samples = deque(maxlen=max(100, int(self.startup_calib_max_samples)))
                self.startup_calib_samples.append(dict(sample))

        # 连续 RF 梯度/源定位样本仍做轻量去重，避免原地 RSSI 抖动长期占满全局缓存。
        append_global = True
        if len(self.rf_samples) > 0:
            last = self.rf_samples[-1]
            dxy = math.hypot(float(sx) - float(last["x"]), float(sy) - float(last["y"]))
            dt = now - float(last["t"])
            if dxy < float(self.rf_gradient_min_motion) and dt < 2.0:
                append_global = False

        if append_global:
            self.rf_samples.append(sample)
            if self.last_best_rf_sample is None:
                self.last_best_rf_sample = sample
            elif sample["quality"] > float(self.last_best_rf_sample.get("quality", 0.0)):
                self.last_best_rf_sample = sample

        self.rf_latest_quality = float(quality)
        self.source_dirty = True

    def update_rf_gradient_locked(self):
        """
        根据最近一段时间内的 LoRa 样本拟合信号增强方向。
        拟合模型：quality = a * x + b * y + c，其中 (a, b) 即 RF 增强方向。
        """
        if not self.enable_rf_gradient:
            self.rf_gradient_x = 0.0
            self.rf_gradient_y = 0.0
            self.rf_gradient_conf = 0.0
            self.rf_gradient_trend = 0.0
            self.rf_search_mode = "COVERAGE"
            return

        now = time.time()
        window = float(self.rf_gradient_window_sec)
        samples = [s for s in self.rf_samples if now - float(s["t"]) <= window]
        self.rf_samples = deque(samples, maxlen=int(self.rf_gradient_max_samples))

        if len(samples) < int(self.rf_gradient_min_samples):
            self.rf_gradient_x = 0.0
            self.rf_gradient_y = 0.0
            self.rf_gradient_conf = 0.0
            self.rf_gradient_trend = 0.0
            self.rf_search_mode = "COVERAGE"
            return

        xs = np.asarray([s["x"] for s in samples], dtype=np.float32)
        ys = np.asarray([s["y"] for s in samples], dtype=np.float32)
        qs = np.asarray([s["quality"] for s in samples], dtype=np.float32)

        spread = math.sqrt(float(np.var(xs) + np.var(ys)))
        if spread < float(self.rf_gradient_min_spread):
            self.rf_gradient_x = 0.0
            self.rf_gradient_y = 0.0
            self.rf_gradient_conf = 0.0
            self.rf_gradient_trend = 0.0
            self.rf_search_mode = "COVERAGE"
            return

        x0 = float(np.mean(xs))
        y0 = float(np.mean(ys))

        A = np.column_stack([xs - x0, ys - y0, np.ones_like(xs)])
        try:
            coef, _residuals, _rank, _singular = np.linalg.lstsq(A, qs, rcond=None)
        except Exception:
            self.rf_gradient_x = 0.0
            self.rf_gradient_y = 0.0
            self.rf_gradient_conf = 0.0
            self.rf_gradient_trend = 0.0
            self.rf_search_mode = "COVERAGE"
            return

        gx = float(coef[0])
        gy = float(coef[1])
        mag = math.hypot(gx, gy)

        if len(qs) >= 6:
            recent_mean = float(np.mean(qs[-3:]))
            prev_mean = float(np.mean(qs[-6:-3]))
            trend = recent_mean - prev_mean
        elif len(qs) >= 3:
            # 低频 LoRa 情况下，样本数少于 6 时也给出趋势估计，
            # 避免出现“有梯度但 trend 永远为 0”的问题。
            trend = float(qs[-1] - qs[0])
        elif len(qs) >= 2:
            trend = float(qs[-1] - qs[-2])
        else:
            trend = 0.0

        spread_score = float(np.clip(spread / max(float(self.rf_gradient_min_spread) * 2.0, 1e-6), 0.0, 1.0))
        grad_score = float(np.clip(mag * float(self.rf_gradient_conf_gain), 0.0, 1.0))
        conf = float(np.clip(grad_score * spread_score, 0.0, 1.0))

        self.rf_gradient_x = gx
        self.rf_gradient_y = gy
        self.rf_gradient_conf = conf
        self.rf_gradient_trend = float(trend)
        self.rf_latest_quality = float(qs[-1])

        if conf < float(self.rf_gradient_conf_threshold):
            mode = "COVERAGE"
        elif self.rf_latest_quality >= float(self.rf_refine_quality_threshold):
            mode = "REFINE_SEARCH"
        elif trend <= float(self.rf_signal_drop_threshold):
            mode = "BACKUP_BRANCH"
        elif trend >= float(self.rf_signal_rise_threshold):
            mode = "GRADIENT_FOLLOW"
        else:
            mode = "COVERAGE"

        self.rf_search_mode = mode

        best_idx = int(np.argmax(qs))
        self.last_best_rf_sample = samples[best_idx]

    def get_source_candidate_mask_flat(self):
        """返回源定位概率图允许的二维栅格掩膜，优先限制在任务区内。"""
        mask = np.ones((self.height, self.width), dtype=bool)

        if self.task_active and self.task_enforce_boundary:
            btype = str(self.task_boundary_type).lower()
            if btype == "rect":
                if self.task_boundary_rect is None:
                    self.normalize_task_boundary()
                xmin, ymin, xmax, ymax = self.task_boundary_rect
                ys, xs = np.indices((self.height, self.width))
                wx = self.origin_x + (xs.astype(np.float32) + 0.5) * float(self.resolution)
                wy = self.origin_y + (ys.astype(np.float32) + 0.5) * float(self.resolution)
                mask = (wx >= xmin) & (wx <= xmax) & (wy >= ymin) & (wy <= ymax)
            elif btype == "polygon" and len(self.task_boundary_polygon) >= 3:
                mask[:, :] = False
                for gy in range(self.height):
                    for gx in range(self.width):
                        wx, wy = self.grid_to_world(gx, gy)
                        mask[gy, gx] = self.point_in_polygon_xy(wx, wy, self.task_boundary_polygon)

        return mask.reshape(-1)

    def select_source_localization_samples_locked(self, now=None):
        """选择参与源定位的最近 LoRa 样本。该函数在 self.lock 内调用。"""
        if now is None:
            now = time.time()
        window = max(float(self.source_window_sec), 1.0)
        samples = [s for s in self.rf_samples if now - float(s["t"]) <= window]

        # 限制最大样本数，保留最近样本，同时保留少量高质量样本。
        max_samples = max(int(self.source_max_samples), int(self.source_min_samples))
        if len(samples) > max_samples:
            recent_keep = max_samples // 2
            recent = samples[-recent_keep:]
            older = samples[:-recent_keep]
            older_sorted = sorted(older, key=lambda s: float(s.get("quality", 0.0)), reverse=True)
            samples = older_sorted[:max_samples - recent_keep] + recent
            samples = sorted(samples, key=lambda s: float(s["t"]))

        return samples

    def update_source_probability_locked(self, now=None, force=False):
        """
        基于多点 LoRa 样本重建信号源概率图。

        核心思想：不直接信任单次 RSSI 绝对值，而是使用相对强弱排序。
        若样本 i 的 quality > 样本 j，则候选源点 S 更应满足 dist(S,i) < dist(S,j)。
        """
        if not bool(self.enable_source_localization):
            return False

        if now is None:
            now = time.time()

        if (not force) and (not self.source_dirty) and (now - float(self.source_last_update_time) < float(self.source_update_interval)):
            return False

        if (not force) and (now - float(self.source_last_update_time) < float(self.source_update_interval)):
            return False

        samples = self.select_source_localization_samples_locked(now)
        self.source_sample_count = int(len(samples))

        if len(samples) < int(self.source_min_samples):
            self.source_probability_map[:, :] = 0.0
            self.source_confidence = 0.0
            self.source_uncertainty_radius = float("inf")
            self.source_peak_probability = 0.0
            self.source_mode = "INSUFFICIENT_SAMPLES"
            self.source_last_update_time = float(now)
            self.source_dirty = False
            return True

        mask_flat = self.get_source_candidate_mask_flat()
        if not np.any(mask_flat):
            self.source_probability_map[:, :] = 0.0
            self.source_confidence = 0.0
            self.source_uncertainty_radius = float("inf")
            self.source_peak_probability = 0.0
            self.source_mode = "NO_VALID_TASK_REGION"
            self.source_last_update_time = float(now)
            self.source_dirty = False
            return True

        ys_idx, xs_idx = np.indices((self.height, self.width))
        wx_all = self.origin_x + (xs_idx.reshape(-1).astype(np.float32) + 0.5) * float(self.resolution)
        wy_all = self.origin_y + (ys_idx.reshape(-1).astype(np.float32) + 0.5) * float(self.resolution)
        wx = wx_all[mask_flat]
        wy = wy_all[mask_flat]
        m = wx.shape[0]
        if m == 0:
            return False

        sx = np.asarray([float(s["x"]) for s in samples], dtype=np.float32)
        sy = np.asarray([float(s["y"]) for s in samples], dtype=np.float32)
        qs = np.asarray([float(s.get("quality", 0.0)) for s in samples], dtype=np.float32)

        # 1) 相对强弱排序一致性评分。
        pairs = []
        min_delta = float(self.source_pair_min_quality_delta)
        for i in range(len(samples)):
            for j in range(len(samples)):
                if qs[i] - qs[j] >= min_delta:
                    pairs.append((i, j, float(qs[i] - qs[j])))

        pair_score = np.zeros((m,), dtype=np.float32)
        if pairs:
            max_pairs = max(1, int(self.source_pair_max_pairs))
            if len(pairs) > max_pairs:
                # 确定性抽样，避免每次刷新图随机跳变。
                step = float(len(pairs)) / float(max_pairs)
                pairs = [pairs[int(k * step)] for k in range(max_pairs)]

            gain = float(self.source_pair_gain)
            dist_scale = max(float(self.source_pair_distance_scale), 1e-6)
            acc = np.zeros((m,), dtype=np.float32)
            for i, j, dq in pairs:
                di = np.hypot(wx - sx[i], wy - sy[i])
                dj = np.hypot(wx - sx[j], wy - sy[j])
                margin = gain * float(dq) * (dj - di) / dist_scale
                margin = np.clip(margin, -30.0, 30.0)
                acc += (1.0 / (1.0 + np.exp(-margin))).astype(np.float32)
            pair_score = acc / float(len(pairs))
        else:
            pair_score[:] = 0.5

        # 2) 高质量样本锚点软约束。它不会单独决定源位置，只是稳定概率峰。
        anchor_score = np.zeros((m,), dtype=np.float32)
        anchor_sigma = max(float(self.source_anchor_sigma), float(self.resolution))
        if len(samples) > 0:
            # 取 top 25% 高质量样本作为锚点。
            k = max(1, int(math.ceil(len(samples) * 0.25)))
            top_idx = np.argsort(qs)[-k:]
            for idx in top_idx:
                d2 = (wx - sx[idx]) ** 2 + (wy - sy[idx]) ** 2
                anchor_score += float(qs[idx]) * np.exp(-d2 / (2.0 * anchor_sigma * anchor_sigma)).astype(np.float32)
            mx = float(np.max(anchor_score))
            if mx > 1e-9:
                anchor_score /= mx

        anchor_w = float(np.clip(self.source_anchor_weight, 0.0, 1.0))
        score = (1.0 - anchor_w) * pair_score + anchor_w * anchor_score

        # 3) 任务先验软融合：任务区内仍可体现最后消失区域/任务片区先验。
        prior_w = float(np.clip(self.source_task_prior_weight, 0.0, 1.0))
        if prior_w > 0.0 and self.task_probability_map.size == self.height * self.width:
            prior_flat = self.task_probability_map.reshape(-1)[mask_flat].astype(np.float32)
            score = (1.0 - prior_w) * score + prior_w * prior_flat

        score = np.clip(score, 0.0, 1.0).astype(np.float32)
        score_full = np.zeros((self.height * self.width,), dtype=np.float32)
        score_full[mask_flat] = score
        prob_map = score_full.reshape((self.height, self.width))

        peak = float(np.max(prob_map)) if prob_map.size else 0.0
        if peak > 1e-9:
            prob_map = prob_map / peak
        else:
            prob_map[:, :] = 0.0

        # 4) 源位置估计：用最高概率附近的加权质心，减少单格跳变。
        flat = prob_map.reshape(-1)
        peak_norm = float(np.max(flat)) if flat.size else 0.0
        if peak_norm <= 1e-9:
            self.source_probability_map = prob_map.astype(np.float32)
            self.source_confidence = 0.0
            self.source_uncertainty_radius = float("inf")
            self.source_peak_probability = 0.0
            self.source_mode = "NO_PROBABILITY_PEAK"
            self.source_last_update_time = float(now)
            self.source_dirty = False
            return True

        top_ratio = float(np.clip(self.source_estimate_top_ratio, 0.01, 1.0))
        top_mask = flat >= peak_norm * top_ratio
        if not np.any(top_mask):
            best_idx = int(np.argmax(flat))
            est_x = float(wx_all[best_idx])
            est_y = float(wy_all[best_idx])
        else:
            w = flat[top_mask].astype(np.float64)
            est_x = float(np.sum(wx_all[top_mask] * w) / max(np.sum(w), 1e-12))
            est_y = float(np.sum(wy_all[top_mask] * w) / max(np.sum(w), 1e-12))

        # 不确定半径：取概率峰 70% 以上区域的加权标准半径。
        uncert_ratio = float(np.clip(self.source_uncertainty_ratio, 0.05, 1.0))
        conf_mask = flat >= peak_norm * uncert_ratio
        if np.any(conf_mask):
            w = flat[conf_mask].astype(np.float64)
            dx = wx_all[conf_mask] - est_x
            dy = wy_all[conf_mask] - est_y
            radius = float(math.sqrt(np.sum(w * (dx * dx + dy * dy)) / max(np.sum(w), 1e-12)))
        else:
            radius = float("inf")

        # 置信度：结合熵集中度、峰值分离度、样本数量。
        prob_sum = float(np.sum(flat))
        if prob_sum > 1e-9:
            p = flat / prob_sum
            p_nonzero = p[p > 1e-12]
            entropy = -float(np.sum(p_nonzero * np.log(p_nonzero)))
            entropy_norm = entropy / max(math.log(float(len(p_nonzero))), 1e-9)
            entropy_conf = float(np.clip(1.0 - entropy_norm, 0.0, 1.0))
        else:
            entropy_conf = 0.0

        p80 = float(np.percentile(flat[mask_flat], 80.0)) if np.any(mask_flat) else 0.0
        peak_margin_conf = float(np.clip((peak_norm - p80) / max(peak_norm, 1e-6), 0.0, 1.0))
        sample_conf = float(np.clip(len(samples) / max(float(self.source_confidence_min_samples_full), 1.0), 0.0, 1.0))
        radius_conf = float(np.clip(1.0 - radius / max(float(self.source_refine_uncertainty_radius) * 2.0, 1e-6), 0.0, 1.0)) if math.isfinite(radius) else 0.0
        confidence = float(np.clip(0.35 * entropy_conf + 0.30 * peak_margin_conf + 0.20 * sample_conf + 0.15 * radius_conf, 0.0, 1.0))

        if confidence >= float(self.source_refine_confidence_threshold) and radius <= float(self.source_refine_uncertainty_radius):
            mode = "REFINE_SEARCH"
        elif confidence >= float(self.source_score_min_confidence):
            mode = "LOCALIZE"
        else:
            mode = "COVERAGE"

        self.source_probability_map = prob_map.astype(np.float32)
        self.source_estimate_x = float(est_x)
        self.source_estimate_y = float(est_y)
        self.source_estimate_z = float(self.route_send_default_z if self.route_send_force_z else max(self.search_z_min, min(self.search_z_max, self.current_z)))
        self.source_confidence = float(confidence)
        self.source_uncertainty_radius = float(radius)
        self.source_peak_probability = float(peak_norm)
        self.source_mode = mode
        self.source_last_update_time = float(now)
        self.source_dirty = False
        return True

    def get_source_probability_score(self, x, y):
        if not bool(self.enable_source_localization):
            return 0.0
        idx = self.world_to_grid(x, y)
        if idx is None:
            return 0.0
        gx, gy = idx
        return float(np.clip(self.source_probability_map[gy, gx], 0.0, 1.0))

    def get_source_probability_multiplier(self, x, y):
        if not bool(self.enable_source_localization):
            return 1.0
        if not bool(self.source_affect_score):
            return 1.0
        if float(self.source_confidence) < float(self.source_score_min_confidence):
            return 1.0
        s = self.get_source_probability_score(x, y)
        mult = 1.0 + float(self.source_multiplier_weight) * float(self.source_confidence) * (2.0 * s - 1.0)
        return float(np.clip(mult, float(self.source_multiplier_min), float(self.source_multiplier_max)))

    def make_source_estimate_status_json(self):
        data = {
            "event": "source_estimate_status",
            "enabled": bool(self.enable_source_localization),
            "mode": str(self.source_mode),
            "sample_count": int(self.source_sample_count),
            "estimate": {
                "x": round(float(self.source_estimate_x), 3),
                "y": round(float(self.source_estimate_y), 3),
                "z": round(float(self.source_estimate_z), 3)
            },
            "confidence": round(float(self.source_confidence), 3),
            "uncertainty_radius": None if not math.isfinite(float(self.source_uncertainty_radius)) else round(float(self.source_uncertainty_radius), 3),
            "peak_probability": round(float(self.source_peak_probability), 3),
            "last_update_age": round(max(0.0, time.time() - float(self.source_last_update_time)), 2) if self.source_last_update_time > 0.0 else None,
            "source_probability_topic": self.source_probability_topic,
            "local_route_trigger_topic": self.source_local_route_trigger_topic,
            "suggestion": self.source_route_suggestion()
        }
        return json.dumps(data, ensure_ascii=False)

    def source_route_suggestion(self):
        if not bool(self.enable_source_localization):
            return "SOURCE_LOCALIZATION_DISABLED"
        if int(self.source_sample_count) < int(self.source_min_samples):
            return "COLLECT_MORE_LORA_SAMPLES"
        if self.source_mode == "REFINE_SEARCH":
            return "SEND_LOCAL_LAWNMOWER_ROUTE_AROUND_SOURCE_ESTIMATE"
        if self.source_mode == "LOCALIZE":
            return "REFRESH_MAIN_ROUTE_TOWARD_SOURCE_PROBABILITY_PEAK"
        return "KEEP_COVERAGE_AND_COLLECT_SAMPLES"

    def publish_source_estimate_status_and_marker(self):
        self.pub_source_estimate_status.publish(String(data=self.make_source_estimate_status_json()))

        markers = MarkerArray()
        stamp = rospy.Time.now()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if (not bool(self.enable_source_localization)) or float(self.source_confidence) <= 0.0:
            self.pub_source_estimate_marker.publish(markers)
            return

        # 估计点球体
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.ns = "source_estimate"
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.source_estimate_x)
        m.pose.position.y = float(self.source_estimate_y)
        m.pose.position.z = float(self.source_estimate_z)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.45
        m.scale.y = 0.45
        m.scale.z = 0.45
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.0
        m.color.a = 0.95
        markers.markers.append(m)

        # 不确定半径圆盘/柱体
        if math.isfinite(float(self.source_uncertainty_radius)):
            r = max(float(self.source_uncertainty_radius), 0.25)
            u = Marker()
            u.header.stamp = stamp
            u.header.frame_id = self.frame_id
            u.ns = "source_uncertainty"
            u.id = 2
            u.type = Marker.CYLINDER
            u.action = Marker.ADD
            u.pose.position.x = float(self.source_estimate_x)
            u.pose.position.y = float(self.source_estimate_y)
            u.pose.position.z = float(self.source_estimate_z) - 0.05
            u.pose.orientation.w = 1.0
            u.scale.x = 2.0 * r
            u.scale.y = 2.0 * r
            u.scale.z = 0.04
            u.color.r = 1.0
            u.color.g = 0.4
            u.color.b = 0.0
            u.color.a = 0.20
            markers.markers.append(u)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = self.frame_id
        text.ns = "source_text"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(self.source_estimate_x)
        text.pose.position.y = float(self.source_estimate_y)
        text.pose.position.z = float(self.source_estimate_z) + 0.8
        text.pose.orientation.w = 1.0
        text.scale.z = 0.45
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.text = "SRC %.2f R=%.1fm" % (float(self.source_confidence), float(self.source_uncertainty_radius) if math.isfinite(float(self.source_uncertainty_radius)) else -1.0)
        markers.markers.append(text)

        self.pub_source_estimate_marker.publish(markers)

    def build_local_lawnmower_goals_around_source(self):
        """围绕 source_estimate 生成局部弓字形搜索点。返回 [(x,y,z,score), ...]。"""
        if float(self.source_confidence) < float(self.source_refine_confidence_threshold):
            return []

        cx = float(self.source_estimate_x)
        cy = float(self.source_estimate_y)
        z = self.route_send_sanitize_z(float(self.source_estimate_z))
        size = max(float(self.source_local_search_size), float(self.source_local_search_spacing))
        spacing = max(float(self.source_local_search_spacing), 0.1)
        half = 0.5 * size

        xs = np.arange(cx - half, cx + half + 1e-6, spacing)
        ys = np.arange(cy - half, cy + half + 1e-6, spacing)

        goals = []
        reverse = False
        for y in ys:
            row = list(xs)
            if reverse:
                row = list(reversed(row))
            for x in row:
                if self.task_active and self.task_enforce_boundary and (not self.point_in_task_boundary(float(x), float(y))):
                    continue
                # 只做轻量安全检查；最终局部避障仍由 EGO Planner 执行。
                if self.has_obstacle_map and self.get_flight_safety_score(float(x), float(y), float(z), self.obstacle_voxels) <= 0.0:
                    continue
                goals.append((float(x), float(y), float(z), 1.0))
                if len(goals) >= int(self.source_local_search_max_goals):
                    return goals
            reverse = not reverse
        return goals

    def build_tcp_local_lawnmower_packet(self):
        goals_raw = self.build_local_lawnmower_goals_around_source()
        if not goals_raw:
            return None

        route_type = str(self.source_local_search_route_type or "LOCAL_SEARCH_ROUTE")
        route_points = []
        for i, (x, y, z, score) in enumerate(goals_raw, start=1):
            route_points.append((float(x), float(y), float(z), float(score), "%02d" % i, "local_lawnmower"))
        route_points = self.route_send_append_return_points(route_points, route_type)

        goals = []
        for i, (x, y, z, score, label, kind) in enumerate(route_points, start=1):
            # yaw 朝向下一点。
            if i < len(route_points):
                nx, ny = route_points[i][0], route_points[i][1]
                yaw = math.atan2(float(ny) - float(y), float(nx) - float(x))
            elif i > 1:
                px, py = route_points[i - 2][0], route_points[i - 2][1]
                yaw = math.atan2(float(y) - float(py), float(x) - float(px))
            else:
                yaw = float(self.current_yaw) if self.has_odom else 0.0
            hold_sec = float(self.route_send_return_hold_sec) if str(kind).startswith("return") else float(self.source_local_search_hold_sec)
            goals.append({
                "seq": int(i),
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "z": round(float(z), 4),
                "yaw": round(float(yaw), 4),
                "hold": hold_sec,
                "name": "src_%s_%02d" % (str(kind), int(i)),
                "source": "dk2500_source_probability",
                "score": round(float(score), 4),
                "label": str(label),
                "kind": str(kind)
            })

        return {
            "cmd": "ROUTE_UPDATE",
            "route_type": route_type,
            "route_id": "source_lawnmower_%d" % int(time.time()),
            "mode": "LAWNMOWER",
            "frame_id": str(self.route_send_frame_id or self.frame_id),
            "preempt": False,
            "goals": goals,
            "source": "dk2500_source_probability",
            "meta": {
                "source_x": round(float(self.source_estimate_x), 4),
                "source_y": round(float(self.source_estimate_y), 4),
                "source_confidence": round(float(self.source_confidence), 4),
                "uncertainty_radius": None if not math.isfinite(float(self.source_uncertainty_radius)) else round(float(self.source_uncertainty_radius), 4),
                "sample_count": int(self.source_sample_count),
                "append_return_path": bool(self.route_send_append_return_path),
                "return_to_current_position": bool(self.route_send_return_to_current_position)
            }
        }

    def source_local_route_trigger_callback(self, msg):
        if not bool(msg.data):
            return
        if not bool(self.enable_tcp_route_sender):
            self.route_send_publish_status("source_local_route_not_sent", {"reason": "tcp_sender_disabled"})
            return
        with self.lock:
            self.update_source_probability_locked(time.time(), force=True)
            packet = self.build_tcp_local_lawnmower_packet()
        if packet is None:
            self.route_send_publish_status("source_local_route_not_sent", {
                "reason": "no_confident_source_or_no_safe_goals",
                "source_confidence": round(float(self.source_confidence), 3),
                "sample_count": int(self.source_sample_count),
                "uncertainty_radius": None if not math.isfinite(float(self.source_uncertainty_radius)) else round(float(self.source_uncertainty_radius), 3)
            })
            rospy.logwarn("No local lawnmower route generated. source_conf=%.3f samples=%d", float(self.source_confidence), int(self.source_sample_count))
            return
        self.tcp_send_json_packet(packet, event_prefix="source_local_lawnmower")


    def startup_calib_reset_callback(self, msg):
        if not bool(msg.data):
            return
        with self.lock:
            self.startup_calib_active = False
            self.startup_calib_completed = False
            self.startup_calib_start_time = 0.0
            self.startup_calib_finish_time = 0.0
            self.startup_calib_result = None
            if hasattr(self, "startup_calib_samples"):
                self.startup_calib_samples.clear()
            self.startup_calib_refresh_sent = False
            self.startup_calib_initial_x = None
            self.startup_calib_initial_y = None
            self.startup_calib_initial_z = None
            self.startup_calib_initial_yaw = None
            self.startup_calib_initial_pose_time = 0.0
            self.startup_calib_reset_point_indicator_locked([], time.time(), reason="manual_reset")
            self.startup_calib_last_status = {"state": "IDLE", "reason": "manual_reset"}
        self.publish_startup_calib_status_and_markers()
        rospy.loginfo("Startup RF calibration reset.")

    def startup_calib_start_callback(self, msg):
        """触发开局 LoRa 方向校准。"""
        if not bool(msg.data):
            return
        if not bool(self.enable_startup_rf_calibration):
            self.startup_calib_publish_status("DISABLED", {"reason": "enable_startup_rf_calibration_false"})
            return
        if not self.has_odom:
            self.startup_calib_publish_status("NO_ODOM", {"reason": "odom_not_ready"})
            rospy.logwarn("Startup RF calibration rejected: odom not ready.")
            return

        packet = None
        with self.lock:
            now = time.time()

            # 先锁定触发瞬间的飞机初始位姿。
            # 之后中心点和十字轴方向都以这个位姿为基准，避免自动选点把 P0 挪开。
            self.capture_startup_calib_initial_pose_locked(now)

            center = self.find_startup_calib_center_locked()
            self.startup_calib_center = center
            self.startup_calib_center_score = float(center.get("score", 0.0))
            self.startup_calib_center_reason = str(center.get("reason", "selected"))
            route = self.build_startup_calib_route_locked(float(center["x"]), float(center["y"]), float(center["z"]))
            self.startup_calib_route_points = route
            self.startup_calib_reset_point_indicator_locked(route, now, reason="start")
            self.startup_calib_active = True
            self.startup_calib_completed = False
            self.startup_calib_start_time = float(now)
            self.startup_calib_finish_time = 0.0
            self.startup_calib_result = None
            if hasattr(self, "startup_calib_samples"):
                self.startup_calib_samples.clear()
            else:
                self.startup_calib_samples = deque(maxlen=max(100, int(self.startup_calib_max_samples)))
            self.startup_calib_refresh_sent = False

            if bool(self.startup_calib_clear_rf_on_start):
                self.rf_samples.clear()
                self.rf_map[:, :] = 0.0
                self.rf_gradient_x = 0.0
                self.rf_gradient_y = 0.0
                self.rf_gradient_conf = 0.0
                self.rf_gradient_trend = 0.0
                self.rf_search_mode = "COVERAGE"
                self.last_best_rf_sample = None
                self.source_dirty = True

            self.startup_calib_last_status = {
                "state": "ACTIVE",
                "reason": "route_generated",
                "start_time": round(float(now), 3),
                "center": self.startup_calib_center_to_json(center),
                "route_count": int(len(route)),
            }

            if bool(self.startup_calib_send_route_on_start):
                packet = self.build_tcp_startup_calib_packet_locked()

        self.publish_startup_calib_status_and_markers()

        if packet is not None:
            self.tcp_send_json_packet(packet, event_prefix="startup_rf_calibration")

        rospy.loginfo("Startup RF calibration started. center=(%.2f, %.2f, %.2f) route_points=%d",
                      float(self.startup_calib_center["x"]),
                      float(self.startup_calib_center["y"]),
                      float(self.startup_calib_center["z"]),
                      len(self.startup_calib_route_points))

    def capture_startup_calib_initial_pose_locked(self, now=None):
        """记录触发开局校准瞬间的飞机 odom 位姿。"""
        if now is None:
            now = time.time()
        self.startup_calib_initial_x = float(self.current_x)
        self.startup_calib_initial_y = float(self.current_y)
        self.startup_calib_initial_z = float(self.current_z)
        self.startup_calib_initial_yaw = float(self.current_yaw)
        self.startup_calib_initial_pose_time = float(now)

    def startup_calib_initial_pose_to_json(self):
        if self.startup_calib_initial_x is None or self.startup_calib_initial_y is None:
            return None
        yaw = 0.0 if self.startup_calib_initial_yaw is None else float(self.startup_calib_initial_yaw)
        return {
            "x": round(float(self.startup_calib_initial_x), 3),
            "y": round(float(self.startup_calib_initial_y), 3),
            "z": None if self.startup_calib_initial_z is None else round(float(self.startup_calib_initial_z), 3),
            "yaw_rad": round(float(yaw), 4),
            "yaw_deg": round(float(math.degrees(yaw)), 1),
            "stamp": round(float(self.startup_calib_initial_pose_time), 3),
        }

    def get_startup_calib_anchor_pose_locked(self):
        """
        返回校准锚点位姿。
        active/completed 阶段优先使用 /startup_rf_calib_start 触发瞬间的初始位姿；
        preview 阶段没有初始位姿时使用当前 odom。
        """
        x = float(self.current_x)
        y = float(self.current_y)
        z = float(self.current_z)
        yaw = float(self.current_yaw) if self.has_odom else 0.0

        if self.startup_calib_initial_x is not None and self.startup_calib_initial_y is not None:
            x = float(self.startup_calib_initial_x)
            y = float(self.startup_calib_initial_y)
        if self.startup_calib_initial_z is not None:
            z = float(self.startup_calib_initial_z)
        if self.startup_calib_initial_yaw is not None:
            yaw = float(self.startup_calib_initial_yaw)

        return x, y, z, yaw

    def startup_calib_center_to_json(self, center):
        if center is None:
            return None
        return {
            "x": round(float(center.get("x", 0.0)), 3),
            "y": round(float(center.get("y", 0.0)), 3),
            "z": round(float(center.get("z", 0.0)), 3),
            "score": round(float(center.get("score", 0.0)), 3),
            "min_clearance": None if center.get("min_clearance", None) is None else round(float(center.get("min_clearance")), 3),
            "obstacle_count": int(center.get("obstacle_count", 0)),
            "reason": str(center.get("reason", "")),
            "direction_source": str(center.get("direction_source", "")),
            "anchor_pose": self.startup_calib_initial_pose_to_json(),
        }

    def get_task_center_xy_locked(self):
        if self.task_active:
            btype = str(self.task_boundary_type).lower()
            if btype == "rect":
                if self.task_boundary_rect is None:
                    self.normalize_task_boundary()
                xmin, ymin, xmax, ymax = self.task_boundary_rect
                return 0.5 * (float(xmin) + float(xmax)), 0.5 * (float(ymin) + float(ymax))
            if btype == "polygon" and len(self.task_boundary_polygon) >= 3:
                xs = [float(p[0]) for p in self.task_boundary_polygon]
                ys = [float(p[1]) for p in self.task_boundary_polygon]
                return float(np.mean(xs)), float(np.mean(ys))
        return None

    def get_startup_calib_direction_locked(self, cx, cy):
        # 手动/开局校准默认锁定十字轴方向到触发 start 瞬间的飞机 yaw。
        # F 方向 = 初始机头方向；B 为反向；L/R 为初始机头左/右侧。
        if bool(self.startup_calib_axis_lock_to_initial_yaw):
            _ax, _ay, _az, yaw = self.get_startup_calib_anchor_pose_locked()
            return math.cos(float(yaw)), math.sin(float(yaw)), "initial_yaw_locked"

        task_center = self.get_task_center_xy_locked()
        if task_center is not None:
            tx, ty = task_center
            dx = float(tx) - float(cx)
            dy = float(ty) - float(cy)
            n = math.hypot(dx, dy)
            if n > 1.0:
                return dx / n, dy / n, "task_center"

        yaw = float(self.current_yaw) if self.has_odom else 0.0
        return math.cos(yaw), math.sin(yaw), "current_yaw"

    def parse_startup_sector_angles_deg(self):
        """解析 centered_sector 的相对角度列表，单位 deg，相对于前向方向。"""
        text = str(self.startup_calib_sector_angles_deg).strip()
        if text:
            vals = []
            for item in text.replace(";", ",").split(","):
                item = item.strip()
                if not item:
                    continue
                try:
                    vals.append(float(item))
                except Exception:
                    pass
            if vals:
                return vals

        count = max(1, int(self.startup_calib_sector_count))
        span = max(0.0, min(360.0, float(self.startup_calib_sector_span_deg)))
        if count <= 1 or span <= 1e-6:
            return [0.0]
        if span >= 360.0 - 1e-6:
            return [i * 360.0 / float(count) for i in range(count)]
        return [-0.5 * span + i * span / float(count - 1) for i in range(count)]

    def startup_calib_route_offsets(self):
        """
        返回 (label, forward, lateral, kind)。
        forward/lateral 是相对校准中心的局部坐标：
          forward 沿“中心 -> 任务区中心”方向；lateral 为其左侧。
        """
        pattern = str(getattr(self, "startup_calib_pattern", "centered_cross")).lower()

        if pattern in ("forward_double_cross", "forward", "double_cross_old"):
            return [
                ("P0", 0.0, 0.0, "center"),
                ("C1", float(self.startup_calib_forward1), 0.0, "centerline"),
                ("L1", float(self.startup_calib_forward1), float(self.startup_calib_lateral1), "left_probe"),
                ("R1", float(self.startup_calib_forward1), -float(self.startup_calib_lateral1), "right_probe"),
                ("C1B", float(self.startup_calib_forward1), 0.0, "centerline_return"),
                ("C2", float(self.startup_calib_forward2), 0.0, "centerline"),
                ("L2", float(self.startup_calib_forward2), float(self.startup_calib_lateral2), "left_probe"),
                ("R2", float(self.startup_calib_forward2), -float(self.startup_calib_lateral2), "right_probe"),
                ("C2B", float(self.startup_calib_forward2), 0.0, "centerline_return"),
            ]

        radius = max(float(self.startup_calib_probe_radius), 0.5)
        ret_center = bool(self.startup_calib_return_center_between_probes)

        if pattern in ("centered_sector", "sector", "fan"):
            offsets = [("P0", 0.0, 0.0, "calib_center")]
            angles = self.parse_startup_sector_angles_deg()
            for i, deg in enumerate(angles, start=1):
                a = math.radians(float(deg))
                fwd = radius * math.cos(a)
                lat = radius * math.sin(a)
                offsets.append(("S%02d" % i, fwd, lat, "sector_probe_%.1fdeg" % float(deg)))
                if ret_center and i != len(angles):
                    offsets.append(("C%02d" % i, 0.0, 0.0, "center_return"))
            if ret_center:
                offsets.append(("CEND", 0.0, 0.0, "center_end"))
            return offsets

        # 默认 centered_cross：以自动选择出的开阔中心为圆心做十字采样。
        probes = [
            ("F", radius, 0.0, "front_probe"),
            ("B", -radius, 0.0, "back_probe"),
            ("L", 0.0, radius, "left_probe"),
            ("R", 0.0, -radius, "right_probe"),
        ]
        if bool(self.startup_calib_include_diagonal_cross):
            diag = radius / math.sqrt(2.0)
            probes.extend([
                ("FL", diag, diag, "front_left_probe"),
                ("FR", diag, -diag, "front_right_probe"),
                ("BL", -diag, diag, "back_left_probe"),
                ("BR", -diag, -diag, "back_right_probe"),
            ])

        offsets = [("P0", 0.0, 0.0, "calib_center")]
        for i, item in enumerate(probes, start=1):
            offsets.append(item)
            if ret_center and i != len(probes):
                offsets.append(("C%d" % i, 0.0, 0.0, "center_return"))
        if ret_center:
            offsets.append(("CEND", 0.0, 0.0, "center_end"))
        return offsets

    def build_startup_calib_route_locked(self, cx, cy, cz):
        fx, fy, reason = self.get_startup_calib_direction_locked(cx, cy)
        lx, ly = -fy, fx
        route = []
        pattern = str(getattr(self, "startup_calib_pattern", "centered_cross")).lower()
        for label, fwd, lat, kind in self.startup_calib_route_offsets():
            x = float(cx) + float(fwd) * fx + float(lat) * lx
            y = float(cy) + float(fwd) * fy + float(lat) * ly
            z = float(cz)
            route.append({
                "label": str(label),
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "hold": float(self.startup_calib_hold_sec),
                "kind": str(kind),
                "forward": float(fwd),
                "lateral": float(lat),
                "pattern": str(pattern),
                "radius": float(math.hypot(float(fwd), float(lat))),
                "direction_source": str(reason),
            })
        return route

    def startup_calib_obstacle_stats_locked(self, x, y, z, radius=None):
        radius = float(self.startup_calib_open_radius if radius is None else radius)
        if (not self.has_obstacle_map) or (not self.obstacle_voxels):
            return {"score": 0.55, "min_clearance": None, "obstacle_count": 0, "reason": "NO_OBSTACLE_MAP_FALLBACK"}

        z = float(z)
        vertical_margin = max(float(self.startup_calib_vertical_margin), 0.1)
        min_d = float("inf")
        count = 0
        for key in self.obstacle_voxels:
            wx, wy, wz = self.voxel_to_point(key)
            if abs(float(wz) - z) > vertical_margin:
                continue
            d = math.hypot(float(wx) - float(x), float(wy) - float(y))
            if d < min_d:
                min_d = d
            if d <= radius:
                count += 1

        if not math.isfinite(min_d):
            min_d = radius * 2.0

        min_clearance = float(self.startup_calib_min_clearance)
        if min_d < min_clearance:
            score = -1.0
            reason = "TOO_CLOSE_TO_OBSTACLE"
        else:
            clearance_score = float(np.clip((min_d - min_clearance) / max(radius - min_clearance, 1e-6), 0.0, 1.0))
            density_penalty = float(np.clip(count / 30.0, 0.0, 1.0)) * 0.35
            score = float(np.clip(0.65 + 0.35 * clearance_score - density_penalty, 0.0, 1.0))
            reason = "OPEN_ENOUGH"

        return {
            "score": float(score),
            "min_clearance": float(min_d),
            "obstacle_count": int(count),
            "reason": str(reason),
        }

    def score_startup_calib_center_locked(self, cx, cy, cz):
        route = self.build_startup_calib_route_locked(cx, cy, cz)
        scores = []
        min_clearance = float("inf")
        obstacle_count = 0
        outside_map = False

        for p in route:
            if self.world_to_grid(float(p["x"]), float(p["y"])) is None:
                outside_map = True
                scores.append(-1.0)
                continue
            st = self.startup_calib_obstacle_stats_locked(float(p["x"]), float(p["y"]), float(p["z"]), radius=float(self.startup_calib_open_radius))
            scores.append(float(st["score"]))
            obstacle_count += int(st.get("obstacle_count", 0))
            if st.get("min_clearance", None) is not None:
                min_clearance = min(min_clearance, float(st["min_clearance"]))

        if not scores:
            return {"score": -1.0, "min_clearance": None, "obstacle_count": 0, "reason": "NO_ROUTE_POINTS"}

        route_score = float(np.mean(scores))
        dist_penalty = 0.02 * math.hypot(float(cx) - float(self.current_x), float(cy) - float(self.current_y))
        score = route_score - dist_penalty
        if outside_map:
            score -= 0.5
            reason = "ROUTE_PARTLY_OUTSIDE_MAP"
        elif route_score < 0.0:
            reason = "ROUTE_NOT_OPEN"
        else:
            reason = "ROUTE_OPEN_ENOUGH"

        return {
            "score": float(score),
            "min_clearance": None if not math.isfinite(min_clearance) else float(min_clearance),
            "obstacle_count": int(obstacle_count),
            "reason": str(reason),
        }

    def find_startup_calib_center_locked(self):
        """
        生成开局 RF 校准中心。

        当前版本默认行为：
          - CENTER/P0 的 x、y 固定为触发 /startup_rf_calib_start 瞬间的飞机 odom 位置；
          - 若 startup_calib_center_use_initial_odom_z=True，则 z 也使用触发瞬间 odom.z；
          - 不再在周围搜索更开阔的中心点。

        这样 RViz 中的 CALIB CENTER/P0 与飞机初始点重合，便于手动抱机校准和复现实验。
        """
        z = float(self.startup_calib_z)
        if bool(self.route_send_force_z):
            z = float(self.route_send_default_z)
        elif self.has_odom and math.isfinite(float(self.current_z)):
            z = float(np.clip(float(self.current_z), float(self.search_z_min), float(self.search_z_max)))

        ax, ay, az, yaw = self.get_startup_calib_anchor_pose_locked()

        if bool(self.startup_calib_center_lock_to_initial_odom):
            cx0 = float(ax)
            cy0 = float(ay)
            if bool(self.startup_calib_center_use_initial_odom_z) and math.isfinite(float(az)):
                z = float(az)

            st = self.score_startup_calib_center_locked(cx0, cy0, z)
            return {
                "x": cx0,
                "y": cy0,
                "z": float(z),
                "score": float(st.get("score", 0.0)),
                "min_clearance": st.get("min_clearance", None),
                "obstacle_count": int(st.get("obstacle_count", 0)),
                "reason": "CENTER_LOCKED_TO_INITIAL_ODOM_%s" % str(st.get("reason", "")),
                "direction_source": "initial_yaw_locked" if bool(self.startup_calib_axis_lock_to_initial_yaw) else "current_logic",
                "initial_yaw_deg": round(float(math.degrees(yaw)), 1),
            }

        # 兼容旧逻辑：只有关闭 center_lock_to_initial_odom 时才在当前点附近搜索开阔中心。
        cx0 = float(self.current_x)
        cy0 = float(self.current_y)

        if not bool(self.startup_calib_auto_select_center):
            st = self.score_startup_calib_center_locked(cx0, cy0, z)
            return {
                "x": cx0, "y": cy0, "z": z,
                "score": float(st.get("score", 0.0)),
                "min_clearance": st.get("min_clearance", None),
                "obstacle_count": int(st.get("obstacle_count", 0)),
                "reason": "AUTO_SELECT_DISABLED_CURRENT_POINT",
                "direction_source": "initial_yaw_locked" if bool(self.startup_calib_axis_lock_to_initial_yaw) else "current_logic",
                "initial_yaw_deg": round(float(math.degrees(yaw)), 1),
            }

        candidates = [(cx0, cy0)]
        radius = max(float(self.startup_calib_search_radius), 0.0)
        step = max(float(self.startup_calib_candidate_step), 0.5)
        if radius > 0.1:
            n = int(math.ceil(radius / step))
            for ix in range(-n, n + 1):
                for iy in range(-n, n + 1):
                    x = cx0 + ix * step
                    y = cy0 + iy * step
                    if math.hypot(x - cx0, y - cy0) <= radius + 1e-6:
                        candidates.append((x, y))

        best = None
        for x, y in candidates:
            if self.world_to_grid(x, y) is None:
                continue
            st = self.score_startup_calib_center_locked(x, y, z)
            item = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "score": float(st.get("score", -1.0)),
                "min_clearance": st.get("min_clearance", None),
                "obstacle_count": int(st.get("obstacle_count", 0)),
                "reason": str(st.get("reason", "")),
                "direction_source": "initial_yaw_locked" if bool(self.startup_calib_axis_lock_to_initial_yaw) else "current_logic",
                "initial_yaw_deg": round(float(math.degrees(yaw)), 1),
            }
            if best is None or item["score"] > best["score"]:
                best = item

        if best is None:
            return {
                "x": cx0, "y": cy0, "z": z,
                "score": 0.0,
                "min_clearance": None,
                "obstacle_count": 0,
                "reason": "NO_VALID_CANDIDATE_CURRENT_POINT",
                "direction_source": "initial_yaw_locked" if bool(self.startup_calib_axis_lock_to_initial_yaw) else "current_logic",
                "initial_yaw_deg": round(float(math.degrees(yaw)), 1),
            }

        return best

    def startup_calib_update_preview_locked(self, now=None):
        if now is None:
            now = time.time()
        if self.startup_calib_active:
            return
        if now - float(self.startup_calib_last_preview_time) < float(self.startup_calib_preview_interval):
            return
        if not self.has_odom:
            return
        center = self.find_startup_calib_center_locked()
        self.startup_calib_center = center
        self.startup_calib_center_score = float(center.get("score", 0.0))
        self.startup_calib_center_reason = str(center.get("reason", "preview"))
        self.startup_calib_route_points = self.build_startup_calib_route_locked(float(center["x"]), float(center["y"]), float(center["z"]))
        self.startup_calib_reset_point_indicator_locked(self.startup_calib_route_points, now, reason="preview")
        self.startup_calib_last_preview_time = float(now)
        self.startup_calib_last_status = {
            "state": "PREVIEW",
            "reason": "selected_open_calibration_center",
            "center": self.startup_calib_center_to_json(center),
            "route_count": int(len(self.startup_calib_route_points)),
        }

    def build_tcp_startup_calib_packet_locked(self):
        if not self.startup_calib_route_points:
            return None
        goals = []
        pts = list(self.startup_calib_route_points)
        for i, p in enumerate(pts, start=1):
            x = float(p["x"])
            y = float(p["y"])
            z = self.route_send_sanitize_z(float(p["z"]))
            if i < len(pts):
                nx, ny = float(pts[i]["x"]), float(pts[i]["y"])
                yaw = math.atan2(ny - y, nx - x)
            elif i > 1:
                px, py = float(pts[i - 2]["x"]), float(pts[i - 2]["y"])
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = float(self.current_yaw) if self.has_odom else 0.0
            goals.append({
                "seq": int(i),
                "x": round(x, 4),
                "y": round(y, 4),
                "z": round(z, 4),
                "yaw": round(float(yaw), 4),
                "hold": float(p.get("hold", self.startup_calib_hold_sec)),
                "name": "startup_%s" % str(p.get("label", i)),
                "label": str(p.get("label", i)),
                "kind": str(p.get("kind", "calib")),
                "source": "dk2500_startup_rf_calibration",
            })
        return {
            "cmd": "ROUTE_UPDATE",
            "route_type": str(self.startup_calib_route_type),
            "route_id": "startup_rf_calib_%d" % int(time.time()),
            "frame_id": str(self.route_send_frame_id or self.frame_id),
            "preempt": False,
            "goals": goals,
            "source": "dk2500_startup_rf_calibration",
            "meta": {
                "center": self.startup_calib_center_to_json(self.startup_calib_center),
                "expected_duration_sec": float(self.startup_calib_expected_duration_sec),
                "purpose": "initial_lora_rf_direction_calibration",
                "pattern": str(getattr(self, "startup_calib_pattern", "centered_cross")),
                "probe_radius": float(getattr(self, "startup_calib_probe_radius", 0.0)),
            }
        }

    def select_startup_calib_samples_locked(self, now=None):
        """
        选择本次开局校准使用的 LoRa 样本。

        逻辑修正：active/completed 的开局校准优先使用 self.startup_calib_samples。
        该缓存从 /startup_rf_calib_start 后开始记录，不按 expected_duration_sec*2 裁剪，
        避免手动校准时间过长时，前面 P0/F/B/L/R 点位的 LoRa 样本过期归零。
        """
        if now is None:
            now = time.time()
        start_time = float(self.startup_calib_start_time)
        if start_time <= 0.0:
            return []

        if hasattr(self, "startup_calib_samples") and (self.startup_calib_active or self.startup_calib_completed):
            samples = [s for s in self.startup_calib_samples if float(s.get("t", 0.0)) >= start_time]
            if bool(getattr(self, "startup_calib_keep_all_active_samples", True)):
                return samples

        # 兼容旧逻辑：没有专用缓存或未启用 keep_all 时，才回退到全局 rf_samples 的时间窗口。
        max_age = max(float(self.startup_calib_expected_duration_sec) * 2.0, 1.0)
        return [
            s for s in self.rf_samples
            if float(s.get("t", 0.0)) >= start_time and now - float(s.get("t", 0.0)) <= max_age
        ]

    def startup_calib_route_signature(self, route=None):
        """生成校准路线签名，用于判断预览/启动后路线是否变化。"""
        if route is None:
            route = self.startup_calib_route_points
        parts = []
        for p in route or []:
            parts.append("%s:%.3f:%.3f:%.3f" % (
                str(p.get("label", "")),
                float(p.get("x", 0.0)),
                float(p.get("y", 0.0)),
                float(p.get("z", 0.0))
            ))
        return "|".join(parts)

    def startup_calib_reset_point_indicator_locked(self, route=None, now=None, reason="reset"):
        """
        重置每个校准点的停留计时状态。
        satisfied=True 的点会在 RViz 中标绿；所有点 satisfied 后输出 ALL CALIB POINTS READY。
        """
        if now is None:
            now = time.time()
        if route is None:
            route = self.startup_calib_route_points

        required = max(float(self.startup_calib_indicator_required_sec), 0.1)
        radius = max(float(self.startup_calib_indicator_radius), 0.05)
        states = []
        for i, p in enumerate(route or []):
            states.append({
                "seq": int(i + 1),
                "label": str(p.get("label", i + 1)),
                "x": float(p.get("x", 0.0)),
                "y": float(p.get("y", 0.0)),
                "z": float(p.get("z", 0.0)),
                "required_sec": float(required),
                "radius": float(radius),
                "inside": False,
                "inside_since": 0.0,
                "dwell_sec": 0.0,
                "satisfied": False,
                "last_distance_xy": None,
                "last_distance_z": None,
                "reason": str(reason),
            })

        self.startup_calib_point_states = states
        self.startup_calib_point_state_route_signature = self.startup_calib_route_signature(route)
        self.startup_calib_all_points_ready = False
        self.startup_calib_last_indicator_update_time = float(now)

    def startup_calib_update_point_indicator_locked(self, now=None):
        """根据当前 odom 更新各校准点停留状态。只有 active 时才累计停留时间。"""
        if now is None:
            now = time.time()
        if not bool(self.startup_calib_indicator_enable):
            return

        route_sig = self.startup_calib_route_signature(self.startup_calib_route_points)
        if route_sig != str(self.startup_calib_point_state_route_signature):
            self.startup_calib_reset_point_indicator_locked(self.startup_calib_route_points, now, reason="route_changed")

        if not self.startup_calib_point_states:
            self.startup_calib_all_points_ready = False
            return

        if not self.has_odom:
            self.startup_calib_all_points_ready = False
            return

        # 预览阶段只显示点，不累计。启动后 active=True 才开始计时。
        if not self.startup_calib_active:
            return

        cx = float(self.current_x)
        cy = float(self.current_y)
        cz = float(self.current_z)
        check_z = bool(self.startup_calib_indicator_check_z)
        z_tol = max(float(self.startup_calib_indicator_z_tolerance), 0.01)

        for st in self.startup_calib_point_states:
            dx = cx - float(st["x"])
            dy = cy - float(st["y"])
            dz = cz - float(st["z"])
            dxy = math.hypot(dx, dy)
            z_ok = (abs(dz) <= z_tol) if check_z else True
            inside = (dxy <= float(st["radius"])) and z_ok

            st["last_distance_xy"] = float(dxy)
            st["last_distance_z"] = float(abs(dz))

            # 已经变绿的点保持 satisfied=True，但 inside/distance 仍持续更新，方便判断当前飞机在哪个点。
            if bool(st.get("satisfied", False)):
                st["inside"] = bool(inside)
                st["dwell_sec"] = float(st["required_sec"])
                st["reason"] = "HOLD_TIME_OK"
                continue

            if inside:
                if not bool(st.get("inside", False)):
                    st["inside"] = True
                    st["inside_since"] = float(now)
                    st["dwell_sec"] = 0.0
                else:
                    st["dwell_sec"] = max(0.0, float(now) - float(st.get("inside_since", now)))

                if float(st["dwell_sec"]) >= float(st["required_sec"]):
                    st["satisfied"] = True
                    st["inside"] = True
                    st["dwell_sec"] = float(st["required_sec"])
                    st["reason"] = "HOLD_TIME_OK"
            else:
                # 要求连续停留；离开半径后本次计时清零，但已经变绿的点不会被取消。
                st["inside"] = False
                st["inside_since"] = 0.0
                st["dwell_sec"] = 0.0
                st["reason"] = "WAITING_FOR_ODOM_IN_POINT"

        self.startup_calib_all_points_ready = all(bool(st.get("satisfied", False)) for st in self.startup_calib_point_states)
        self.startup_calib_last_indicator_update_time = float(now)

    def startup_calib_indicator_summary_locked(self):
        """输出给 /startup_rf_calib_indicator 和状态 JSON 的校准点停留摘要。"""
        states = []
        ready_count = 0
        for st in self.startup_calib_point_states:
            satisfied = bool(st.get("satisfied", False))
            if satisfied:
                ready_count += 1
            states.append({
                "seq": int(st.get("seq", 0)),
                "label": str(st.get("label", "")),
                "x": round(float(st.get("x", 0.0)), 3),
                "y": round(float(st.get("y", 0.0)), 3),
                "z": round(float(st.get("z", 0.0)), 3),
                "satisfied": satisfied,
                "inside": bool(st.get("inside", False)),
                "dwell_sec": round(float(st.get("dwell_sec", 0.0)), 2),
                "required_sec": round(float(st.get("required_sec", self.startup_calib_indicator_required_sec)), 2),
                "radius": round(float(st.get("radius", self.startup_calib_indicator_radius)), 2),
                "last_distance_xy": None if st.get("last_distance_xy", None) is None else round(float(st.get("last_distance_xy")), 3),
                "last_distance_z": None if st.get("last_distance_z", None) is None else round(float(st.get("last_distance_z")), 3),
                "reason": str(st.get("reason", "")),
            })

        total = int(len(self.startup_calib_point_states))
        return {
            "enabled": bool(self.startup_calib_indicator_enable),
            "all_points_ready": bool(self.startup_calib_all_points_ready),
            "ready_count": int(ready_count),
            "total_count": int(total),
            "required_sec": round(float(self.startup_calib_indicator_required_sec), 2),
            "radius": round(float(self.startup_calib_indicator_radius), 2),
            "check_z": bool(self.startup_calib_indicator_check_z),
            "z_tolerance": round(float(self.startup_calib_indicator_z_tolerance), 2),
            "require_all_points_ready_to_finish": bool(self.startup_calib_require_all_points_ready_to_finish),
            "states": states,
        }

    def startup_calib_required_lora_label_list(self):
        """解析需要 LoRa 样本达标的校准点标签，默认 P0/F/B/L/R。"""
        text = str(getattr(self, "startup_calib_required_lora_labels", "P0,F,B,L,R")).strip()
        labels = []
        for item in text.replace(";", ",").split(","):
            item = item.strip()
            if item and item not in labels:
                labels.append(item)
        return labels if labels else ["P0", "F", "B", "L", "R"]

    def startup_calib_required_lora_route_points(self):
        """
        返回需要积攒 LoRa 样本的校准点。
        默认只取 P0/F/B/L/R，避免 C1/C2/CEND 等回中心点重复参与门槛判断。
        """
        labels = self.startup_calib_required_lora_label_list()
        label_set = set(labels)
        out = []
        seen = set()
        for p in self.startup_calib_route_points:
            label = str(p.get("label", ""))
            if label in label_set and label not in seen:
                out.append(p)
                seen.add(label)

        # 非 centered_cross 模式下，如果用户配置的标签不存在，则退化为所有非回中心点。
        if not out:
            for p in self.startup_calib_route_points:
                kind = str(p.get("kind", ""))
                label = str(p.get("label", ""))
                if "return" in kind or "end" in kind:
                    continue
                if label not in seen:
                    out.append(p)
                    seen.add(label)
        return out

    def startup_calib_lora_point_summary_locked(self, samples, return_fit_samples=False):
        """
        将 LoRa 样本唯一分配给最近的必需校准点，并统计每个点是否达到样本数门槛。

        注意：这里采用“最近点唯一分配”，不是对每个点按半径重复计数。
        这样可以避免 sample_assignment_radius 较大时，一个 LoRa 样本同时被 P0/F/B/L/R 重复计入。
        """
        points = self.startup_calib_required_lora_route_points()
        radius = max(float(self.startup_calib_sample_assignment_radius), 0.1)
        min_per_point = max(1, int(getattr(self, "startup_calib_min_lora_per_point", 1)))

        buckets = {}
        point_by_label = {}
        for p in points:
            label = str(p.get("label", ""))
            buckets[label] = []
            point_by_label[label] = p

        unassigned = 0
        for s in samples:
            best_label = None
            best_d = float("inf")
            sx = float(s.get("x", 0.0))
            sy = float(s.get("y", 0.0))
            for p in points:
                d = math.hypot(sx - float(p.get("x", 0.0)), sy - float(p.get("y", 0.0)))
                if d < best_d:
                    best_d = d
                    best_label = str(p.get("label", ""))
            if best_label is not None and best_d <= radius:
                buckets[best_label].append(s)
            else:
                unassigned += 1

        point_summaries = []
        ready_count = 0
        fit_samples = []
        for p in points:
            label = str(p.get("label", ""))
            vals = buckets.get(label, [])
            vals_sorted = sorted(vals, key=lambda item: float(item.get("t", 0.0)))
            fit_samples.extend(vals_sorted)
            count = int(len(vals_sorted))
            ready = count >= min_per_point
            if ready:
                ready_count += 1
            qualities = [float(v.get("quality", 0.0)) for v in vals_sorted]
            rssis = [float(v.get("rssi", v.get("effective_rssi", 0.0))) for v in vals_sorted]
            raw_rssis = [float(v.get("raw_rssi", v.get("rssi", 0.0))) for v in vals_sorted]
            point_summaries.append({
                "label": label,
                "x": round(float(p.get("x", 0.0)), 3),
                "y": round(float(p.get("y", 0.0)), 3),
                "required_count": int(min_per_point),
                "count": int(count),
                "ready": bool(ready),
                "quality_median": None if not qualities else round(float(np.median(qualities)), 3),
                "rssi_median": None if not rssis else round(float(np.median(rssis)), 1),
                "raw_rssi_median": None if not raw_rssis else round(float(np.median(raw_rssis)), 1),
            })

        total = int(len(points))
        summary = {
            "enabled": bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True)),
            "labels": [str(p.get("label", "")) for p in points],
            "min_lora_per_point": int(min_per_point),
            "assignment_radius": round(float(radius), 3),
            "ready_count": int(ready_count),
            "total_count": int(total),
            "all_ready": bool(total > 0 and ready_count == total),
            "fit_sample_count": int(len(fit_samples)),
            "unassigned_sample_count": int(unassigned),
            "points": point_summaries,
        }
        if return_fit_samples:
            return summary, sorted(fit_samples, key=lambda item: float(item.get("t", 0.0)))
        return summary

    def startup_calib_lora_points_ready_locked(self, samples=None, now=None):
        if samples is None:
            if now is None:
                now = time.time()
            samples = self.select_startup_calib_samples_locked(now)
        summary = self.startup_calib_lora_point_summary_locked(samples, return_fit_samples=False)
        return bool(summary.get("all_ready", False)), summary

    def compute_startup_calib_result_locked(self, now=None):
        if now is None:
            now = time.time()
        samples = self.select_startup_calib_samples_locked(now)
        lora_point_summary, fit_samples = self.startup_calib_lora_point_summary_locked(samples, return_fit_samples=True)
        require_lora_points = bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True))
        samples_for_fit = fit_samples if require_lora_points else list(samples)
        result = {
            "state": "ACTIVE" if self.startup_calib_active else ("COMPLETED" if self.startup_calib_completed else "PREVIEW"),
            "sample_count": int(len(samples)),
            "fit_sample_count": int(len(samples_for_fit)),
            "confidence": 0.0,
            "heading_deg": 0.0,
            "gradient": {"x": 0.0, "y": 0.0},
            "spread": 0.0,
            "trend": 0.0,
            "suggestion": "COLLECT_MORE_LORA_SAMPLES",
            "waypoints": lora_point_summary.get("points", []),
            "lora_point_summary": lora_point_summary,
            "point_indicator": self.startup_calib_indicator_summary_locked(),
        }

        if require_lora_points and not bool(lora_point_summary.get("all_ready", False)):
            result["suggestion"] = "WAIT_FOR_EACH_CALIB_POINT_LORA_SAMPLE"
            self.startup_calib_result = result
            return result

        if len(samples_for_fit) < int(self.startup_calib_min_samples):
            result["suggestion"] = "COLLECT_MORE_LORA_SAMPLES_FOR_FIT"
            self.startup_calib_result = result
            return result

        xs = np.asarray([float(s["x"]) for s in samples_for_fit], dtype=np.float32)
        ys = np.asarray([float(s["y"]) for s in samples_for_fit], dtype=np.float32)
        qs = np.asarray([float(s.get("quality", 0.0)) for s in samples_for_fit], dtype=np.float32)
        spread = math.sqrt(float(np.var(xs) + np.var(ys)))
        result["spread"] = round(float(spread), 3)
        if spread < float(self.startup_calib_min_spread):
            result["suggestion"] = "MOVE_MORE_TO_CREATE_RF_BASELINE"
            self.startup_calib_result = result
            return result

        x0 = float(np.mean(xs))
        y0 = float(np.mean(ys))
        A = np.column_stack([xs - x0, ys - y0, np.ones_like(xs)])
        try:
            coef, _residuals, _rank, _singular = np.linalg.lstsq(A, qs, rcond=None)
            gx = float(coef[0])
            gy = float(coef[1])
        except Exception:
            gx, gy = 0.0, 0.0

        mag = math.hypot(gx, gy)
        spread_score = float(np.clip(spread / max(float(self.startup_calib_min_spread) * 2.0, 1e-6), 0.0, 1.0))
        grad_score = float(np.clip(mag * float(self.startup_calib_conf_gain), 0.0, 1.0))
        sample_score = float(np.clip(len(samples_for_fit) / max(float(self.startup_calib_min_samples) * 1.5, 1.0), 0.0, 1.0))
        conf = float(np.clip(0.55 * grad_score + 0.25 * spread_score + 0.20 * sample_score, 0.0, 1.0))
        if len(qs) >= 4:
            trend = float(np.mean(qs[-2:]) - np.mean(qs[:2]))
        elif len(qs) >= 2:
            trend = float(qs[-1] - qs[0])
        else:
            trend = 0.0

        heading_deg = math.degrees(math.atan2(gy, gx)) if mag > 1e-9 else 0.0
        result.update({
            "confidence": round(float(conf), 3),
            "heading_deg": round(float(heading_deg), 1),
            "gradient": {"x": round(float(gx), 6), "y": round(float(gy), 6)},
            "trend": round(float(trend), 4),
            "latest_quality": round(float(qs[-1]), 3),
            "min_quality": round(float(np.min(qs)), 3),
            "max_quality": round(float(np.max(qs)), 3),
        })
        if conf >= float(self.startup_calib_conf_threshold):
            result["suggestion"] = "INITIAL_RF_DIRECTION_VALID_REFRESH_MAIN_ROUTE"
        else:
            result["suggestion"] = "INITIAL_RF_DIRECTION_WEAK_REPEAT_OR_USE_COVERAGE"
        self.startup_calib_result = result
        return result

    def startup_calib_route_quality_summary(self, samples):
        out = []
        if not self.startup_calib_route_points:
            return out
        radius = max(float(self.startup_calib_sample_assignment_radius), 0.1)
        for p in self.startup_calib_route_points:
            vals = []
            for s in samples:
                if math.hypot(float(s["x"]) - float(p["x"]), float(s["y"]) - float(p["y"])) <= radius:
                    vals.append(float(s.get("quality", 0.0)))
            out.append({
                "label": str(p.get("label", "")),
                "x": round(float(p.get("x", 0.0)), 3),
                "y": round(float(p.get("y", 0.0)), 3),
                "count": int(len(vals)),
                "quality_median": None if not vals else round(float(np.median(vals)), 3),
            })
        return out

    def maybe_finish_startup_calibration_locked(self, now=None):
        if now is None:
            now = time.time()
        if not self.startup_calib_active:
            return
        result = self.compute_startup_calib_result_locked(now)
        elapsed = float(now) - float(self.startup_calib_start_time)
        conf = float(result.get("confidence", 0.0))
        enough_time = elapsed >= float(self.startup_calib_expected_duration_sec)
        early_ok = bool(self.startup_calib_auto_finish_on_conf) and elapsed >= float(self.startup_calib_min_duration_sec) and conf >= float(self.startup_calib_conf_threshold)
        if bool(self.startup_calib_require_all_points_ready_to_finish) and not bool(self.startup_calib_all_points_ready):
            self.startup_calib_last_status = {
                "state": "ACTIVE",
                "elapsed": round(float(elapsed), 1),
                "reason": "waiting_for_all_calib_points_ready",
                "center": self.startup_calib_center_to_json(self.startup_calib_center),
                "result": result,
                "point_indicator": self.startup_calib_indicator_summary_locked(),
                "lora_point_summary": result.get("lora_point_summary", {}),
            }
            return
        if bool(getattr(self, "startup_calib_require_lora_per_point_to_finish", True)) and not bool(result.get("lora_point_summary", {}).get("all_ready", False)):
            self.startup_calib_last_status = {
                "state": "ACTIVE",
                "elapsed": round(float(elapsed), 1),
                "reason": "waiting_for_lora_samples_at_all_required_calib_points",
                "center": self.startup_calib_center_to_json(self.startup_calib_center),
                "result": result,
                "point_indicator": self.startup_calib_indicator_summary_locked(),
                "lora_point_summary": result.get("lora_point_summary", {}),
            }
            return
        if not (enough_time or early_ok):
            self.startup_calib_last_status = {
                "state": "ACTIVE",
                "elapsed": round(float(elapsed), 1),
                "center": self.startup_calib_center_to_json(self.startup_calib_center),
                "result": result,
                "point_indicator": self.startup_calib_indicator_summary_locked(),
            }
            return

        self.startup_calib_active = False
        self.startup_calib_completed = True
        self.startup_calib_finish_time = float(now)
        result["state"] = "COMPLETED"
        result["elapsed"] = round(float(elapsed), 1)
        result["center"] = self.startup_calib_center_to_json(self.startup_calib_center)
        result["point_indicator"] = self.startup_calib_indicator_summary_locked()
        self.startup_calib_result = result
        self.startup_calib_last_status = {"state": "COMPLETED", "result": result}

        # 校准方向作为初始 RF 先验，避免主路线刷新时完全依赖瞬时 rf_gradient_status。
        lora_points_ready = bool(result.get("lora_point_summary", {}).get("all_ready", False))
        init_rf_allowed = conf >= float(self.startup_calib_conf_threshold) and (
            (not bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True))) or lora_points_ready
        )

        if bool(self.startup_calib_use_as_initial_rf_prior) and init_rf_allowed:
            self.rf_gradient_x = float(result["gradient"]["x"])
            self.rf_gradient_y = float(result["gradient"]["y"])
            self.rf_gradient_conf = max(float(self.rf_gradient_conf), conf)
            self.rf_search_mode = "GRADIENT_FOLLOW"

        if init_rf_allowed and not self.startup_calib_refresh_sent:
            with self.queue_lock:
                if bool(self.startup_calib_auto_refresh_main):
                    self.ordered_goal_reset_requested = True
                    self.selected_backup_route_id = None
                if bool(self.startup_calib_auto_refresh_backup):
                    self.backup_routes_refresh_requested = True
                    self.cached_backup_valid = False
                    self.cached_backup_action = "startup_rf_calib_refresh_requested"
            self.startup_calib_refresh_sent = True

    def handle_startup_calibration_timer(self, now=None):
        if not bool(self.enable_startup_rf_calibration):
            return
        if now is None:
            now = time.time()
        with self.lock:
            if not self.startup_calib_active and not self.startup_calib_completed:
                self.startup_calib_update_preview_locked(now)
            if self.startup_calib_active:
                self.startup_calib_update_point_indicator_locked(now)
                self.maybe_finish_startup_calibration_locked(now)
        self.publish_startup_calib_status_and_markers()

    def startup_calib_status_json(self):
        result = self.startup_calib_result if isinstance(self.startup_calib_result, dict) else None
        data = {
            "event": "startup_rf_calib_status",
            "enabled": bool(self.enable_startup_rf_calibration),
            "active": bool(self.startup_calib_active),
            "completed": bool(self.startup_calib_completed),
            "center": self.startup_calib_center_to_json(self.startup_calib_center),
            "center_reason": str(self.startup_calib_center_reason),
            "initial_pose": self.startup_calib_initial_pose_to_json(),
            "center_lock_to_initial_odom": bool(self.startup_calib_center_lock_to_initial_odom),
            "axis_lock_to_initial_yaw": bool(self.startup_calib_axis_lock_to_initial_yaw),
            "center_use_initial_odom_z": bool(self.startup_calib_center_use_initial_odom_z),
            "route_count": int(len(self.startup_calib_route_points)),
            "pattern": str(getattr(self, "startup_calib_pattern", "centered_cross")),
            "probe_radius": round(float(getattr(self, "startup_calib_probe_radius", 0.0)), 3),
            "start_topic": self.startup_calib_start_topic,
            "reset_topic": self.startup_calib_reset_topic,
            "path_topic": self.startup_calib_path_topic,
            "marker_topic": self.startup_calib_marker_topic,
            "indicator_topic": self.startup_calib_indicator_topic,
            "point_indicator": self.startup_calib_indicator_summary_locked(),
            "lora_point_summary": self.startup_calib_lora_point_summary_locked(self.select_startup_calib_samples_locked(time.time()), return_fit_samples=False),
            "startup_sample_buffer_count": int(len(self.startup_calib_samples)) if hasattr(self, "startup_calib_samples") else 0,
            "startup_sample_keep_all_active": bool(getattr(self, "startup_calib_keep_all_active_samples", True)),
            "result": result,
            "last_status": self.startup_calib_last_status,
        }
        if self.startup_calib_active and self.startup_calib_start_time > 0.0:
            data["elapsed"] = round(float(time.time() - self.startup_calib_start_time), 1)
        if self.startup_calib_completed and self.startup_calib_finish_time > 0.0:
            data["finish_age"] = round(float(time.time() - self.startup_calib_finish_time), 1)
        return json.dumps(data, ensure_ascii=False)

    def startup_calib_publish_status(self, state, extra=None):
        if extra is None:
            extra = {}
        data = {"event": "startup_rf_calib_status", "state": str(state), "extra": extra}
        self.pub_startup_calib_status.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def publish_startup_calib_status_and_markers(self):
        if not bool(self.enable_startup_rf_calibration):
            return
        status_text = self.startup_calib_status_json()
        self.pub_startup_calib_status.publish(String(data=status_text))
        self.pub_startup_calib_indicator.publish(String(data=json.dumps(self.startup_calib_indicator_summary_locked(), ensure_ascii=False)))
        if self.startup_calib_completed and self.startup_calib_result is not None:
            self.pub_startup_calib_result.publish(String(data=json.dumps(self.startup_calib_result, ensure_ascii=False)))
        self.publish_startup_calib_path()
        self.publish_startup_calib_markers()

    def publish_startup_calib_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        for p in self.startup_calib_route_points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p["x"])
            ps.pose.position.y = float(p["y"])
            ps.pose.position.z = float(p["z"])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_startup_calib_path.publish(path)

    def publish_startup_calib_markers(self):
        markers = MarkerArray()
        stamp = rospy.Time.now()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        mid = 1
        if self.startup_calib_center is not None:
            cx = float(self.startup_calib_center["x"])
            cy = float(self.startup_calib_center["y"])
            cz = float(self.startup_calib_center["z"])

            c = Marker()
            c.header.stamp = stamp
            c.header.frame_id = self.frame_id
            c.ns = "startup_rf_calib_center"
            c.id = mid; mid += 1
            c.type = Marker.SPHERE
            c.action = Marker.ADD
            c.pose.position.x = cx
            c.pose.position.y = cy
            c.pose.position.z = cz
            c.pose.orientation.w = 1.0
            c.scale.x = 0.7
            c.scale.y = 0.7
            c.scale.z = 0.7
            c.color.r = 0.1
            c.color.g = 0.7
            c.color.b = 1.0
            c.color.a = 0.95
            c.lifetime = rospy.Duration(0.0)
            markers.markers.append(c)

            ring = Marker()
            ring.header.stamp = stamp
            ring.header.frame_id = self.frame_id
            ring.ns = "startup_rf_calib_open_radius"
            ring.id = mid; mid += 1
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = cx
            ring.pose.position.y = cy
            ring.pose.position.z = cz - 0.08
            ring.pose.orientation.w = 1.0
            r = max(float(self.startup_calib_open_radius), 0.5)
            ring.scale.x = 2.0 * r
            ring.scale.y = 2.0 * r
            ring.scale.z = 0.04
            ring.color.r = 0.1
            ring.color.g = 0.7
            ring.color.b = 1.0
            ring.color.a = 0.16
            ring.lifetime = rospy.Duration(0.0)
            markers.markers.append(ring)

            txt = Marker()
            txt.header.stamp = stamp
            txt.header.frame_id = self.frame_id
            txt.ns = "startup_rf_calib_center_text"
            txt.id = mid; mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = cx
            txt.pose.position.y = cy
            txt.pose.position.z = cz + 1.1
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.55
            txt.color.r = 0.1
            txt.color.g = 0.9
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = "CALIB CENTER\n(%.1f, %.1f, %.1f)\n%s R=%.1fm\nscore=%.2f %s" % (
                cx, cy, cz,
                str(getattr(self, "startup_calib_pattern", "centered_cross")),
                float(getattr(self, "startup_calib_probe_radius", 0.0)),
                float(self.startup_calib_center.get("score", 0.0)),
                str(self.startup_calib_center_reason)
            )
            txt.lifetime = rospy.Duration(0.0)
            markers.markers.append(txt)

        if self.startup_calib_route_points:
            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self.frame_id
            line.ns = "startup_rf_calib_route_line"
            line.id = mid; mid += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.r = 0.1
            line.color.g = 0.9
            line.color.b = 1.0
            line.color.a = 0.95
            line.pose.orientation.w = 1.0
            line.lifetime = rospy.Duration(0.0)
            for p in self.startup_calib_route_points:
                pt = Point()
                pt.x = float(p["x"])
                pt.y = float(p["y"])
                pt.z = float(p["z"]) + 0.12
                line.points.append(pt)
            markers.markers.append(line)

            indicator_states = list(self.startup_calib_point_states) if bool(self.startup_calib_indicator_enable) else []
            ready_count = 0
            for i, p in enumerate(self.startup_calib_route_points):
                st = indicator_states[i] if i < len(indicator_states) else None
                satisfied = bool(st.get("satisfied", False)) if st is not None else False
                inside = bool(st.get("inside", False)) if st is not None else False
                dwell = float(st.get("dwell_sec", 0.0)) if st is not None else 0.0
                required = float(st.get("required_sec", self.startup_calib_indicator_required_sec)) if st is not None else float(self.startup_calib_indicator_required_sec)
                radius = float(st.get("radius", self.startup_calib_indicator_radius)) if st is not None else float(self.startup_calib_indicator_radius)
                if satisfied:
                    ready_count += 1

                # 不满足条件时不标绿：灰色=未进入/未停够，橙色=正在计时，绿色=该点已停够。
                if satisfied:
                    cr, cg, cb, ca = 0.0, 1.0, 0.0, 1.0
                    state_text = "OK"
                elif inside:
                    cr, cg, cb, ca = 1.0, 0.58, 0.0, 0.95
                    state_text = "HOLD"
                else:
                    cr, cg, cb, ca = 0.72, 0.72, 0.72, 0.75
                    state_text = "WAIT"

                pm = Marker()
                pm.header.stamp = stamp
                pm.header.frame_id = self.frame_id
                pm.ns = "startup_rf_calib_points"
                pm.id = mid; mid += 1
                pm.type = Marker.SPHERE
                pm.action = Marker.ADD
                pm.pose.position.x = float(p["x"])
                pm.pose.position.y = float(p["y"])
                pm.pose.position.z = float(p["z"])
                pm.pose.orientation.w = 1.0
                pm.scale.x = 0.42
                pm.scale.y = 0.42
                pm.scale.z = 0.42
                pm.color.r = cr
                pm.color.g = cg
                pm.color.b = cb
                pm.color.a = ca
                pm.lifetime = rospy.Duration(0.0)
                markers.markers.append(pm)

                pr = Marker()
                pr.header.stamp = stamp
                pr.header.frame_id = self.frame_id
                pr.ns = "startup_rf_calib_point_hold_radius"
                pr.id = mid; mid += 1
                pr.type = Marker.CYLINDER
                pr.action = Marker.ADD
                pr.pose.position.x = float(p["x"])
                pr.pose.position.y = float(p["y"])
                pr.pose.position.z = float(p["z"]) - 0.06
                pr.pose.orientation.w = 1.0
                pr.scale.x = 2.0 * max(radius, 0.05)
                pr.scale.y = 2.0 * max(radius, 0.05)
                pr.scale.z = 0.03
                pr.color.r = cr
                pr.color.g = cg
                pr.color.b = cb
                pr.color.a = 0.18 if satisfied else (0.13 if inside else 0.08)
                pr.lifetime = rospy.Duration(0.0)
                markers.markers.append(pr)

                tm = Marker()
                tm.header.stamp = stamp
                tm.header.frame_id = self.frame_id
                tm.ns = "startup_rf_calib_point_text"
                tm.id = mid; mid += 1
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = float(p["x"])
                tm.pose.position.y = float(p["y"])
                tm.pose.position.z = float(p["z"]) + 0.7
                tm.pose.orientation.w = 1.0
                tm.scale.z = 0.42
                tm.color.r = cr
                tm.color.g = cg
                tm.color.b = cb
                tm.color.a = 1.0
                tm.text = "%s  %s\n%.1f / %.1fs\n(%.1f, %.1f)" % (
                    str(p["label"]),
                    state_text,
                    min(float(dwell), float(required)),
                    float(required),
                    float(p["x"]),
                    float(p["y"])
                )
                tm.lifetime = rospy.Duration(0.0)
                markers.markers.append(tm)

            if bool(self.startup_calib_indicator_enable) and self.startup_calib_route_points:
                summary = self.startup_calib_indicator_summary_locked()
                all_ready = bool(summary.get("all_points_ready", False))
                total = int(summary.get("total_count", len(self.startup_calib_route_points)))
                ready = int(summary.get("ready_count", ready_count))
                sx = float(self.startup_calib_center["x"]) if self.startup_calib_center is not None else float(self.startup_calib_route_points[0]["x"])
                sy = float(self.startup_calib_center["y"]) if self.startup_calib_center is not None else float(self.startup_calib_route_points[0]["y"])
                sz = float(self.startup_calib_center["z"]) if self.startup_calib_center is not None else float(self.startup_calib_route_points[0]["z"])
                sm = Marker()
                sm.header.stamp = stamp
                sm.header.frame_id = self.frame_id
                sm.ns = "startup_rf_calib_indicator_summary"
                sm.id = mid; mid += 1
                sm.type = Marker.TEXT_VIEW_FACING
                sm.action = Marker.ADD
                sm.pose.position.x = sx
                sm.pose.position.y = sy
                sm.pose.position.z = sz + 2.0
                sm.pose.orientation.w = 1.0
                sm.scale.z = 0.55
                if all_ready:
                    sm.color.r = 0.0
                    sm.color.g = 1.0
                    sm.color.b = 0.0
                    sm.text = "ALL CALIB POINTS READY\n%d/%d GREEN\nnow RF calibration samples are valid" % (ready, total)
                else:
                    sm.color.r = 1.0
                    sm.color.g = 0.65
                    sm.color.b = 0.0
                    sm.text = "CALIB POINT HOLD CHECK\n%d/%d GREEN\nNeed odom stay %.1fs within %.2fm" % (
                        ready,
                        total,
                        float(self.startup_calib_indicator_required_sec),
                        float(self.startup_calib_indicator_radius)
                    )
                sm.color.a = 1.0
                sm.lifetime = rospy.Duration(0.0)
                markers.markers.append(sm)

        result = self.startup_calib_result if isinstance(self.startup_calib_result, dict) else None
        lora_points_ready_for_init = True
        if result is not None and bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True)):
            lora_points_ready_for_init = bool(result.get("lora_point_summary", {}).get("all_ready", False))
        init_rf_marker_ready = (
            result is not None
            and bool(self.startup_calib_completed)
            and bool(lora_points_ready_for_init)
            and float(result.get("confidence", 0.0)) >= float(self.startup_calib_conf_threshold)
            and self.startup_calib_center is not None
        )
        if init_rf_marker_ready:
            gx = float(result.get("gradient", {}).get("x", 0.0))
            gy = float(result.get("gradient", {}).get("y", 0.0))
            gnorm = math.hypot(gx, gy)
            if gnorm > 1e-9:
                cx = float(self.startup_calib_center["x"])
                cy = float(self.startup_calib_center["y"])
                cz = float(self.startup_calib_center["z"])
                length = float(self.startup_calib_arrow_length) * (0.5 + float(result.get("confidence", 0.0)))
                p0 = Point(); p0.x = cx; p0.y = cy; p0.z = cz + 0.4
                p1 = Point(); p1.x = cx + length * gx / gnorm; p1.y = cy + length * gy / gnorm; p1.z = cz + 0.4
                arr = Marker()
                arr.header.stamp = stamp
                arr.header.frame_id = self.frame_id
                arr.ns = "startup_rf_calib_initial_direction"
                arr.id = mid; mid += 1
                arr.type = Marker.ARROW
                arr.action = Marker.ADD
                arr.points = [p0, p1]
                arr.scale.x = 0.12
                arr.scale.y = 0.35
                arr.scale.z = 0.45
                arr.color.r = 1.0
                arr.color.g = 1.0
                arr.color.b = 0.0
                arr.color.a = 1.0
                arr.lifetime = rospy.Duration(0.0)
                markers.markers.append(arr)

                rt = Marker()
                rt.header.stamp = stamp
                rt.header.frame_id = self.frame_id
                rt.ns = "startup_rf_calib_result_text"
                rt.id = mid; mid += 1
                rt.type = Marker.TEXT_VIEW_FACING
                rt.action = Marker.ADD
                rt.pose.position.x = p1.x
                rt.pose.position.y = p1.y
                rt.pose.position.z = p1.z + 0.8
                rt.pose.orientation.w = 1.0
                rt.scale.z = 0.48
                rt.color.r = 1.0
                rt.color.g = 1.0
                rt.color.b = 0.0
                rt.color.a = 1.0
                lps = result.get("lora_point_summary", {}) if isinstance(result, dict) else {}
                rt.text = "INIT RF %.1f deg\nconf=%.2f fit=%d total=%d\nLoRa points %d/%d ready" % (
                    float(result.get("heading_deg", 0.0)),
                    float(result.get("confidence", 0.0)),
                    int(result.get("fit_sample_count", result.get("sample_count", 0))),
                    int(result.get("sample_count", 0)),
                    int(lps.get("ready_count", 0)),
                    int(lps.get("total_count", 0))
                )
                rt.lifetime = rospy.Duration(0.0)
                markers.markers.append(rt)

        self.pub_startup_calib_markers.publish(markers)

    def get_effective_rf_gradient_for_score(self):
        gx = float(self.rf_gradient_x)
        gy = float(self.rf_gradient_y)
        conf = float(self.rf_gradient_conf)
        mode = str(self.rf_search_mode)
        source = "rf_gradient"

        startup_lora_points_ready = True
        if isinstance(self.startup_calib_result, dict) and bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True)):
            startup_lora_points_ready = bool(self.startup_calib_result.get("lora_point_summary", {}).get("all_ready", False))

        if (
            bool(self.startup_calib_use_as_initial_rf_prior) and
            self.startup_calib_completed and
            bool(startup_lora_points_ready) and
            isinstance(self.startup_calib_result, dict) and
            float(self.startup_calib_result.get("confidence", 0.0)) >= float(self.startup_calib_conf_threshold)
        ):
            age = time.time() - float(self.startup_calib_finish_time or time.time())
            if age <= float(self.startup_calib_prior_hold_sec):
                sgx = float(self.startup_calib_result.get("gradient", {}).get("x", 0.0))
                sgy = float(self.startup_calib_result.get("gradient", {}).get("y", 0.0))
                sconf = float(self.startup_calib_result.get("confidence", 0.0))
                if math.hypot(sgx, sgy) > 1e-9 and (conf < float(self.rf_gradient_conf_threshold) or sconf >= conf):
                    gx, gy, conf, mode, source = sgx, sgy, sconf, "GRADIENT_FOLLOW", "startup_rf_calib"
        return gx, gy, conf, mode, source

    def get_rf_gradient_score(self, x, y):
        """
        根据候选点与 RF 梯度方向的一致性返回倍率。
        >1 顺着信号增强方向；=1 中性；<1 背离信号增强方向。
        开局校准完成后，优先使用 startup_rf_calib 的初始方向作为短时 RF 先验。
        """
        startup_lora_points_ready = True
        if isinstance(self.startup_calib_result, dict) and bool(getattr(self, "startup_calib_require_lora_per_point_for_init", True)):
            startup_lora_points_ready = bool(self.startup_calib_result.get("lora_point_summary", {}).get("all_ready", False))
        startup_prior_available = (
            bool(self.startup_calib_use_as_initial_rf_prior) and
            self.startup_calib_completed and
            bool(startup_lora_points_ready) and
            isinstance(self.startup_calib_result, dict) and
            float(self.startup_calib_result.get("confidence", 0.0)) >= float(self.startup_calib_conf_threshold)
        )
        if (not self.enable_rf_gradient) and (not startup_prior_available):
            return 1.0
        if not self.rf_gradient_affect_score:
            return 1.0
        if not self.has_odom:
            return 1.0

        gx, gy, gconf, gmode, _gsource = self.get_effective_rf_gradient_for_score()
        if float(gconf) < float(self.rf_gradient_conf_threshold):
            return 1.0

        gnorm = math.hypot(float(gx), float(gy))
        if gnorm < 1e-6:
            return 1.0

        dx = float(x) - float(self.current_x)
        dy = float(y) - float(self.current_y)
        dnorm = math.hypot(dx, dy)
        if dnorm < 1e-6:
            return 1.0

        align = (dx / dnorm) * (float(gx) / gnorm) + (dy / dnorm) * (float(gy) / gnorm)
        align = float(np.clip(align, -1.0, 1.0))

        weight = float(self.rf_gradient_weight)
        if gmode == "REFINE_SEARCH":
            weight *= 0.6
        elif gmode == "BACKUP_BRANCH":
            weight *= 0.5

        multiplier = 1.0 + weight * float(gconf) * align
        return float(np.clip(
            multiplier,
            float(self.rf_gradient_min_multiplier),
            float(self.rf_gradient_max_multiplier)
        ))

    def rf_gradient_route_suggestion(self):
        if not self.enable_rf_gradient:
            return "RF_GRADIENT_DISABLED"
        if len(self.rf_samples) < int(self.rf_gradient_min_samples):
            return "MOVE_AND_COLLECT_MORE_LORA_SAMPLES"
        if self.rf_gradient_conf < float(self.rf_gradient_conf_threshold):
            return "NO_RELIABLE_GRADIENT_KEEP_COVERAGE_SEARCH"
        if self.rf_search_mode == "GRADIENT_FOLLOW":
            return "REFRESH_MAIN_ROUTE_TOWARD_STRONGER_SIGNAL"
        if self.rf_search_mode == "BACKUP_BRANCH":
            return "REFRESH_BACKUP_ROUTE_FROM_LAST_STRONG_SIGNAL_AREA"
        if self.rf_search_mode == "REFINE_SEARCH":
            return "ENTER_LOCAL_REFINE_SEARCH_AROUND_HIGH_SIGNAL_AREA"
        return "KEEP_COVERAGE_SEARCH"

    def make_rf_gradient_status_json(self):
        heading_deg = 0.0
        if math.hypot(self.rf_gradient_x, self.rf_gradient_y) > 1e-6:
            heading_deg = math.degrees(math.atan2(self.rf_gradient_y, self.rf_gradient_x))

        best = None
        if self.last_best_rf_sample is not None:
            best = {
                "x": round(float(self.last_best_rf_sample["x"]), 3),
                "y": round(float(self.last_best_rf_sample["y"]), 3),
                "z": round(float(self.last_best_rf_sample["z"]), 3),
                "quality": round(float(self.last_best_rf_sample["quality"]), 3),
                "rssi": round(float(self.last_best_rf_sample["rssi"]), 1),
                "effective_rssi": round(float(self.last_best_rf_sample.get("effective_rssi", self.last_best_rf_sample["rssi"])), 1),
                "raw_rssi": round(float(self.last_best_rf_sample.get("raw_rssi", self.last_best_rf_sample["rssi"])), 1),
                "virtual_attenuation_db": round(float(self.last_best_rf_sample.get("virtual_attenuation_db", 0.0)), 1),
                "snr": round(float(self.last_best_rf_sample["snr"]), 2)
            }

        data = {
            "event": "rf_gradient_status",
            "enabled": bool(self.enable_rf_gradient),
            "publish_continuous": bool(self.publish_continuous_rf_gradient),
            "mode": str(self.rf_search_mode),
            "sample_count": int(len(self.rf_samples)),
            "latest_quality": round(float(self.rf_latest_quality), 3),
            "trend": round(float(self.rf_gradient_trend), 4),
            "gradient": {
                "x": round(float(self.rf_gradient_x), 5),
                "y": round(float(self.rf_gradient_y), 5),
                "confidence": round(float(self.rf_gradient_conf), 3),
                "heading_deg": round(float(heading_deg), 1)
            },
            "best_sample": best,
            "lora_virtual_attenuation_db": round(float(self.lora_virtual_attenuation_db), 1),
            "lora_rssi_score_min": round(float(self.lora_rssi_score_min), 1),
            "lora_rssi_score_max": round(float(self.lora_rssi_score_max), 1),
            "lora_snr_score_max": round(float(self.lora_snr_score_max), 1),
            "suggestion": self.rf_gradient_route_suggestion(),
            "source_localization": {
                "enabled": bool(self.enable_source_localization),
                "mode": str(self.source_mode),
                "confidence": round(float(self.source_confidence), 3),
                "estimate_x": round(float(self.source_estimate_x), 3),
                "estimate_y": round(float(self.source_estimate_y), 3),
                "uncertainty_radius": None if not math.isfinite(float(self.source_uncertainty_radius)) else round(float(self.source_uncertainty_radius), 3),
                "sample_count": int(self.source_sample_count)
            },
            "startup_rf_calibration": {
                "enabled": bool(self.enable_startup_rf_calibration),
                "active": bool(self.startup_calib_active),
                "completed": bool(self.startup_calib_completed),
                "center": self.startup_calib_center_to_json(self.startup_calib_center),
                "result": self.startup_calib_result if isinstance(self.startup_calib_result, dict) else None
            }
        }
        return json.dumps(data, ensure_ascii=False)

    def publish_rf_gradient_status_and_marker(self):
        """发布连续 RF 梯度状态/箭头。

        默认不发布连续梯度，避免手动采几个点后 RViz 一直显示 RF Gradient。
        当 publish_continuous_rf_gradient=False 时，只发送 DELETEALL 清理旧箭头，
        不再向 /rf_gradient_status 输出滚动梯度 JSON。
        """
        markers = MarkerArray()
        stamp = rospy.Time.now()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not bool(self.publish_continuous_rf_gradient):
            self.pub_rf_gradient_marker.publish(markers)
            return

        self.pub_rf_gradient_status.publish(String(data=self.make_rf_gradient_status_json()))

        if (
            not self.enable_rf_gradient or
            not self.has_odom or
            self.rf_gradient_conf < float(self.rf_gradient_conf_threshold)
        ):
            self.pub_rf_gradient_marker.publish(markers)
            return

        gx = float(self.rf_gradient_x)
        gy = float(self.rf_gradient_y)
        gnorm = math.hypot(gx, gy)
        if gnorm < 1e-6:
            self.pub_rf_gradient_marker.publish(markers)
            return

        length = 2.5 + 2.0 * float(self.rf_gradient_conf)

        p0 = Point()
        p0.x = float(self.current_x)
        p0.y = float(self.current_y)
        p0.z = float(self.current_z) + 0.35

        p1 = Point()
        p1.x = float(self.current_x) + length * gx / gnorm
        p1.y = float(self.current_y) + length * gy / gnorm
        p1.z = float(self.current_z) + 0.35

        arrow = Marker()
        arrow.header.stamp = stamp
        arrow.header.frame_id = self.frame_id
        arrow.ns = "rf_gradient"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [p0, p1]
        arrow.scale.x = 0.08
        arrow.scale.y = 0.22
        arrow.scale.z = 0.28
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.lifetime = rospy.Duration(0.0)
        markers.markers.append(arrow)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = self.frame_id
        text.ns = "rf_gradient_text"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = p1.x
        text.pose.position.y = p1.y
        text.pose.position.z = p1.z + 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.45
        text.color.r = 0.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "RF Gradient\n%s\nconf=%.2f trend=%.3f" % (
            str(self.rf_search_mode),
            float(self.rf_gradient_conf),
            float(self.rf_gradient_trend)
        )
        text.lifetime = rospy.Duration(0.0)
        markers.markers.append(text)

        self.pub_rf_gradient_marker.publish(markers)

    def get_rf_score(self, x, y, rf_map):
        idx = self.world_to_grid(x, y)
        if idx is None:
            return 0.0

        gx, gy = idx
        return float(rf_map[gy, gx])

    def get_prior_score(self, x, y):
        idx = self.world_to_grid(x, y)
        if idx is None:
            return 0.0

        gx, gy = idx
        return float(self.prior_map[gy, gx])

    def get_wall_score(self, x, y, wall_columns):
        key_xy = self.point_to_xy_voxel(x, y)
        if key_xy in wall_columns:
            return 0.0
        return 1.0

    def get_fov_score(self, x, y):
        if not self.use_fov_filter:
            return 1.0

        if not self.has_odom:
            return 1.0

        dx = x - self.current_x
        dy = y - self.current_y

        if math.hypot(dx, dy) < 1e-3:
            return 1.0

        angle = math.atan2(dy, dx)
        diff = abs(self.normalize_angle(angle - self.current_yaw))

        front_half = math.radians(self.front_fov_deg * 0.5)
        soft_half = math.radians(self.soft_fov_deg * 0.5)

        if diff <= front_half:
            return 1.0

        if diff >= soft_half:
            return float(self.back_score)

        t = (diff - front_half) / max(soft_half - front_half, 1e-6)
        return float(1.0 - t)

    def get_flight_safety_score(self, x, y, z, obstacle_voxels):
        if not obstacle_voxels:
            return 1.0

        vx, vy, vz = self.point_to_voxel(x, y, z)
        nearest_d = None

        for d, ox, oy, oz in self.neighbor_offsets:
            key = (vx + ox, vy + oy, vz + oz)
            if key in obstacle_voxels:
                nearest_d = d
                break

        if nearest_d is None:
            return 1.0

        if nearest_d <= self.hard_collision_radius:
            return 0.0

        if nearest_d >= self.safe_radius:
            return 1.0

        return float((nearest_d - self.hard_collision_radius) / (self.safe_radius - self.hard_collision_radius))

    def get_line_of_sight_score(self, x, y, z, obstacle_voxels):
        if not self.enable_line_of_sight:
            return 1.0

        if not self.has_odom:
            return 1.0

        sx = self.current_x
        sy = self.current_y
        sz = self.current_z

        dx = x - sx
        dy = y - sy
        dz = z - sz

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 1e-6:
            return 1.0

        steps = max(2, int(dist / self.los_step))

        for i in range(1, steps):
            t = float(i) / float(steps)
            traveled = t * dist

            if traveled < self.los_skip_near:
                continue

            px = sx + dx * t
            py = sy + dy * t
            pz = sz + dz * t

            key = self.point_to_voxel(px, py, pz)
            if key in obstacle_voxels:
                return 0.0

        return 1.0

    def get_observed_score(self, x, y, z, free_voxels):
        if not self.enable_observed_filter:
            return 1.0

        if not self.has_odom:
            return 1.0

        if not free_voxels:
            return float(self.observed_unknown_score)

        vx, vy, vz = self.point_to_voxel(x, y, z)

        for ox, oy, oz in self.observed_offsets:
            if (vx + ox, vy + oy, vz + oz) in free_voxels:
                return 1.0

        return float(self.observed_unknown_score)

    def get_terrain_score(self, x, y, z):
        if not self.enable_terrain_filter:
            return 1.0

        idx = self.world_to_grid(x, y)
        if idx is None:
            return 0.0

        gx, gy = idx

        base_score = float(self.terrain_score_map[gy, gx])
        if base_score <= 0.0:
            if self.terrain_soft_filter:
                return float(np.clip(self.terrain_unknown_score, 0.0, 1.0))
            return 0.0

        ground_z = self.ground_z_map[gy, gx]
        if not np.isfinite(ground_z):
            if self.terrain_soft_filter:
                return float(np.clip(self.terrain_unknown_score, 0.0, 1.0))
            return 0.0

        rel_z = float(z - ground_z)

        # 人员可能性高度约束：避免天花板、墙面高处获得高分
        if rel_z < self.human_z_min:
            height_score = 0.2
        elif rel_z <= self.human_z_max:
            height_score = 1.0
        elif rel_z >= self.human_z_soft_max:
            height_score = 0.0
        else:
            t = (
                (rel_z - self.human_z_max) /
                max(self.human_z_soft_max - self.human_z_max, 1e-6)
            )
            height_score = 1.0 - t

        return float(np.clip(base_score * height_score, 0.0, 1.0))

    def apply_terrain_soft_multiplier(self, terrain_score):
        if not self.enable_terrain_filter:
            return 1.0

        terrain_score = float(np.clip(terrain_score, 0.0, 1.0))

        if not self.terrain_soft_filter:
            return terrain_score

        min_mul = float(np.clip(self.terrain_min_multiplier, 0.0, 1.0))
        return float(min_mul + (1.0 - min_mul) * terrain_score)

    def compute_candidate_scores(self, rf_map, obstacle_voxels, wall_columns, free_voxels):
        scores = np.zeros((self.candidates.shape[0],), dtype=np.float32)

        for i, (x, y, z) in enumerate(self.candidates):
            # 搜索任务启用时，候选点必须落在任务边界内。
            if self.task_active and self.task_enforce_boundary:
                if not self.point_in_task_boundary(x, y):
                    scores[i] = 0.0
                    continue

            rf_score = self.get_rf_score(x, y, rf_map)
            source_score = self.get_source_probability_score(x, y)
            if (
                bool(self.enable_source_localization) and
                bool(self.source_affect_score) and
                float(self.source_confidence) >= float(self.source_score_min_confidence)
            ):
                rf_score = max(rf_score, float(self.source_score_weight) * source_score)

            # 没有 LoRa/RF 热点时，用初始概率分布提供初始搜索证据。
            # 这样输入“搜索边界 + 最后已知坐标”后，地图会立即生成初始主路线/备用路线。
            if self.task_active and self.use_initial_probability_for_search:
                init_prob = self.get_task_probability_score(x, y)
                rf_score = max(rf_score, float(self.initial_probability_evidence_weight) * init_prob)

            if rf_score <= 0.001:
                scores[i] = 0.0
                continue

            prior_score = self.get_prior_score(x, y)
            if prior_score <= 0.0:
                scores[i] = 0.0
                continue

            observed_score = self.get_observed_score(x, y, z, free_voxels)
            if observed_score <= 0.0:
                scores[i] = 0.0
                continue

            terrain_score = self.get_terrain_score(x, y, z)
            if terrain_score <= 0.0:
                scores[i] = 0.0
                continue

            terrain_score = self.apply_terrain_soft_multiplier(terrain_score)

            fov_score = self.get_fov_score(x, y)
            if fov_score <= 0.0:
                scores[i] = 0.0
                continue

            wall_score = self.get_wall_score(x, y, wall_columns)
            if wall_score <= 0.0:
                scores[i] = 0.0
                continue

            safety_score = self.get_flight_safety_score(x, y, z, obstacle_voxels)
            if safety_score <= 0.0:
                scores[i] = 0.0
                continue

            los_score = self.get_line_of_sight_score(x, y, z, obstacle_voxels)
            if los_score <= 0.0:
                scores[i] = 0.0
                continue

            gradient_score = self.get_rf_gradient_score(x, y)
            source_multiplier = self.get_source_probability_multiplier(x, y)

            scores[i] = float(np.clip(
                rf_score
                * prior_score
                * observed_score
                * terrain_score
                * fov_score
                * safety_score
                * wall_score
                * los_score
                * gradient_score
                * source_multiplier,
                0.0,
                1.0
            ))

        return scores

    def timer_callback(self, event):
        try:
            now = time.time()
            map_rebuilt = False
            rebuild_stats = None
            recent_points_to_publish = None
            free_voxels_to_publish = None

            with self.lock:
                self.rf_map *= self.rf_decay

                if bool(self.rebuild_map_only_on_interval):
                    interval = max(0.1, float(self.full_map_rebuild_interval))
                    due = (self.last_full_map_rebuild_time <= 0.0) or ((now - self.last_full_map_rebuild_time) >= interval)
                    should_rebuild = due and (self.full_map_rebuild_requested or (not self.has_obstacle_map))

                    if should_rebuild:
                        rebuild_stats, recent_points_to_publish, free_voxels_to_publish = self.perform_full_map_rebuild_locked(now)
                        map_rebuilt = True

                rf = np.copy(self.rf_map)
                obstacle_voxels = set(self.obstacle_voxels)
                wall_columns = set(self.wall_columns)
                free_voxels = set(self.free_voxels)
                has_obstacle_map = self.has_obstacle_map
                terrain_score_map = np.copy(self.terrain_score_map)
                task_probability_map = np.copy(self.task_probability_map)
                prior_map = np.copy(self.prior_map)

                self.update_source_probability_locked(now)
                source_probability_map = np.copy(self.source_probability_map)

                self.fusion_map_2d = np.maximum(self.rf_map, self.source_score_weight * self.source_probability_map) * self.prior_map
                fusion_2d = np.copy(self.fusion_map_2d)

            # 最近障碍点云和 observed free cloud 只在全地图重建时发布，避免 RViz 高频刷新卡顿。
            if map_rebuilt:
                self.publish_recent_obstacle_cloud(recent_points_to_publish)
                self.publish_observed_free_cloud(free_voxels_to_publish)
                rospy.loginfo(
                    "Full map rebuilt: recent_points=%d target=%d raw_seen=%d frames_used=%d cached_frames=%d oldest_age=%.1fs range_counts=[%s] voxels=%d wall_columns=%d free_voxels=%d terrain_max=%.3f frame=%s",
                    int(rebuild_stats["recent_points"]),
                    int(self.recent_target_points),
                    int(rebuild_stats["raw_seen"]),
                    int(rebuild_stats["frames_used"]),
                    int(rebuild_stats["cached_frames"]),
                    float(rebuild_stats["oldest_age"]),
                    str(rebuild_stats["range_counts"]),
                    int(rebuild_stats["voxels"]),
                    int(rebuild_stats["wall_columns"]),
                    int(rebuild_stats["free_voxels"]),
                    float(rebuild_stats["terrain_max"]),
                    str(rebuild_stats["frame_id"])
                )

            self.pub_rf.publish(self.to_grid_msg(rf))
            self.pub_prior.publish(self.to_grid_msg(prior_map))
            self.pub_fusion_2d.publish(self.to_grid_msg(fusion_2d))
            self.pub_terrain_score.publish(self.to_grid_msg(terrain_score_map))
            self.pub_initial_probability.publish(self.to_grid_msg(task_probability_map))
            self.pub_source_probability.publish(self.to_grid_msg(source_probability_map))
            self.publish_task_area_markers()
            self.publish_search_task_status()
            self.publish_rf_gradient_status_and_marker()
            self.publish_source_estimate_status_and_marker()
            self.handle_startup_calibration_timer(now)

            if not has_obstacle_map:
                rospy.logwarn_throttle(2.0, "Recent obstacle map not ready. Waiting for cloud.")
                return

            if now - self.last_score_publish_time < self.score_publish_interval:
                return

            self.last_score_publish_time = now

            scores = self.compute_candidate_scores(rf, obstacle_voxels, wall_columns, free_voxels)

            self.publish_score_cloud(self.candidates, scores)
            self.publish_high_score_goals_and_path(self.candidates, scores)
            self.publish_next_goal(self.candidates, scores)

        except Exception as e:
            rospy.logerr("timer_callback error: %s", str(e))

    def to_grid_msg(self, grid):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        grid = np.clip(grid, 0.0, 1.0)
        msg.data = (grid * 100.0).astype(np.int8).flatten().tolist()

        return msg

    def pack_rgb(self, r, g, b):
        r = int(np.clip(r, 0, 255))
        g = int(np.clip(g, 0, 255))
        b = int(np.clip(b, 0, 255))
        rgb_uint32 = (r << 16) | (g << 8) | b
        return struct.unpack("f", struct.pack("I", rgb_uint32))[0]

    def score_color(self, score):
        score = float(np.clip(score, 0.0, 1.0))

        if score < 0.5:
            t = score / 0.5
            r = 0
            g = int(255 * t)
            b = int(255 * (1.0 - t))
        else:
            t = (score - 0.5) / 0.5
            r = int(255 * t)
            g = int(255 * (1.0 - t))
            b = 0

        return r, g, b

    def publish_score_cloud(self, candidates, scores):
        if candidates.shape[0] == 0:
            return

        n = candidates.shape[0]

        if n > self.max_score_points:
            idx = np.random.choice(n, self.max_score_points, replace=False)
            show_candidates = candidates[idx]
            show_scores = scores[idx]
        else:
            show_candidates = candidates
            show_scores = scores

        cloud_points = []

        for (x, y, z), score in zip(show_candidates, show_scores):
            r, g, b = self.score_color(score)
            rgb = self.pack_rgb(r, g, b)
            cloud_points.append([float(x), float(y), float(z), rgb])

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        msg = pc2.create_cloud(header, fields, cloud_points)
        self.pub_score_cloud.publish(msg)

        rospy.loginfo_throttle(
            2.0,
            "Published UAV 3D score cloud: %d points, max_score=%.3f yaw=%.2f deg",
            len(cloud_points),
            float(np.max(scores)) if scores.size > 0 else 0.0,
            math.degrees(self.current_yaw)
        )

    def dist3(self, a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def limit_goal_step(self, old_goal, new_goal):
        if old_goal is None:
            return new_goal

        dx = new_goal[0] - old_goal[0]
        dy = new_goal[1] - old_goal[1]
        dz = new_goal[2] - old_goal[2]
        d = math.sqrt(dx * dx + dy * dy + dz * dz)

        if d <= self.max_goal_step:
            return new_goal

        scale = self.max_goal_step / max(d, 1e-6)
        return (
            old_goal[0] + dx * scale,
            old_goal[1] + dy * scale,
            old_goal[2] + dz * scale
        )

    def smooth_goal(self, old_goal, new_goal):
        if old_goal is None:
            return new_goal

        a = float(np.clip(self.goal_smooth_alpha, 0.0, 1.0))
        return (
            old_goal[0] * (1.0 - a) + new_goal[0] * a,
            old_goal[1] * (1.0 - a) + new_goal[1] * a,
            old_goal[2] * (1.0 - a) + new_goal[2] * a
        )


    def dist_xy(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def extract_high_score_goals(self, candidates, scores):
        """
        从所有候选搜索视点中提取高分 goal 点。
        返回格式：[(x, y, z, score), ...]，默认按 score 从高到低排列。
        这里做 2D 非极大值抑制，避免同一片高分区域输出一堆密集点。
        """
        if scores.size == 0:
            return [], 0.0, 0.0

        max_score = float(np.max(scores))
        if max_score < self.min_goal_score:
            return [], 0.0, max_score

        ratio = float(np.clip(self.high_goal_score_ratio, 0.0, 1.0))
        threshold = max(float(self.high_goal_min_score), max_score * ratio, float(self.min_goal_score))
        max_num = max(1, int(self.high_goal_max_num))
        min_sep = max(0.0, float(self.high_goal_min_separation))

        order = np.argsort(scores)[::-1]
        goals = []
        cur_pos = None
        if self.has_odom:
            cur_pos = (self.current_x, self.current_y, self.current_z)

        for idx in order:
            score = float(scores[idx])
            if score < threshold:
                break

            x, y, z = candidates[idx]
            goal = (float(x), float(y), float(z), score)

            # 距离当前点太近的点不作为路径点，避免原地抖动。
            if cur_pos is not None:
                if self.dist3(cur_pos, goal[:3]) < self.min_goal_distance:
                    continue

            # 2D 间隔约束：同一个 XY 区域只保留最高分点。
            too_close = False
            for g in goals:
                if self.dist_xy(goal, g) < min_sep:
                    too_close = True
                    break

            if too_close:
                continue

            goals.append(goal)

            if len(goals) >= max_num:
                break

        return goals, threshold, max_score

    def order_goals_for_path(self, goals):
        """
        把高分 goals 排成路径点序列。
        score 模式：按分数从高到低。
        nearest 模式：从当前无人机位置开始做最近邻排序，减少路径来回跳。
        """
        if not goals:
            return []

        mode = str(self.path_order_mode).lower()
        if mode == "score":
            return list(goals)

        remaining = list(goals)
        ordered = []

        if self.has_odom:
            cur = (self.current_x, self.current_y, self.current_z)
        else:
            first = remaining.pop(0)
            ordered.append(first)
            cur = first[:3]

        while remaining:
            best_i = 0
            best_d = None

            for i, g in enumerate(remaining):
                d = self.dist3(cur, g[:3])
                if best_d is None or d < best_d:
                    best_d = d
                    best_i = i

            g = remaining.pop(best_i)
            ordered.append(g)
            cur = g[:3]

        return ordered

    def make_pose_array_msg(self, goals, include_current=False):
        """
        构造 PoseArray。
        注意：PoseArray 没有显式 seq 字段，poses 的数组下标就是先后顺序：
        poses[0] -> 第 1 个 goal，poses[1] -> 第 2 个 goal。
        """
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        if include_current and self.has_odom:
            p0 = Pose()
            p0.position.x = float(self.current_x)
            p0.position.y = float(self.current_y)
            p0.position.z = float(self.current_z)
            p0.orientation.w = 1.0
            msg.poses.append(p0)

        for x, y, z, _score in goals:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.w = 1.0
            msg.poses.append(p)

        return msg

    def publish_high_goal_cloud(self, goals, max_score):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        cloud_points = []
        denom = max(float(max_score), 1e-6)

        for x, y, z, score in goals:
            # 用归一化后的相对分数着色，方便在 RViz 中突出高分目标点。
            color_score = float(np.clip(score / denom, 0.0, 1.0))
            r, g, b = self.score_color(color_score)
            rgb = self.pack_rgb(r, g, b)
            cloud_points.append([float(x), float(y), float(z), rgb])

        msg = pc2.create_cloud(header, fields, cloud_points)
        self.pub_high_goals_cloud.publish(msg)

    def make_path_msg(self, ordered_goals, include_current=None):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id

        if include_current is None:
            include_current = self.path_include_current_position

        # 用完整可视化航线生成 Path：START -> ENTRY -> 01 -> 02 ...
        # 当 include_current=False 时，保持旧行为，只发布搜索点之间的路径。
        route_points = self.build_ordered_visual_route_points(
            ordered_goals,
            include_current=include_current
        )

        for x, y, z, _score, _label, _kind in route_points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        return path


    def publish_waypoint_path(self, ordered_goals):
        path = self.make_path_msg(ordered_goals, include_current=self.path_include_current_position)
        self.pub_waypoint_path.publish(path)

    def publish_ordered_goal_path(self, ordered_goals):
        path = self.make_path_msg(ordered_goals, include_current=self.ordered_goal_include_current_position)
        self.pub_ordered_goal_path.publish(path)

    def make_ordered_goal_sequence_json(self, ordered_goals, threshold, max_score, queue_action=None, queue_value=None):
        seq = []
        for i, (x, y, z, score) in enumerate(ordered_goals, start=1):
            seq.append({
                "seq": int(i),
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "z": round(float(z), 3),
                "score": round(float(score), 4)
            })

        queue_age = 0.0
        if self.stable_ordered_goal_time > 0.0:
            queue_age = max(0.0, time.time() - self.stable_ordered_goal_time)

        entry_point = None
        entry = self.compute_task_entry_point_to_first_goal(ordered_goals)
        if entry is not None:
            ex, ey, ez, _escore, elabel, _ekind = entry
            entry_point = {
                "label": str(elabel),
                "x": round(float(ex), 3),
                "y": round(float(ey), 3),
                "z": round(float(ez), 3)
            }

        data = {
            "event": "ordered_goal_sequence",
            "frame_id": self.frame_id,
            "stamp": rospy.Time.now().to_sec(),
            "order_mode": str(self.path_order_mode),
            "queue_enable": bool(self.ordered_goal_queue_enable),
            "strict_lock": bool(self.ordered_goal_strict_lock),
            "drop_reached_auto": bool(self.ordered_goal_drop_reached),
            "queue_action": str(queue_action if queue_action is not None else self.stable_ordered_goal_last_action),
            "queue_value": round(float(queue_value if queue_value is not None else self.stable_ordered_goal_value), 4),
            "queue_age": round(float(queue_age), 2),
            "manual_refresh": bool(self.routes_manual_refresh),
            "main_refresh_topic": self.main_routes_refresh_topic,
            "backup_refresh_topic": self.backup_routes_refresh_topic,
            "combined_refresh_topic": self.routes_refresh_topic,
            "include_current_in_path": bool(self.ordered_goal_include_current_position),
            "include_current_in_array": bool(self.ordered_goal_array_include_current_position),
            "task_entry_point_enable": bool(self.task_entry_point_enable),
            "entry_point": entry_point,
            "num_goals": int(len(ordered_goals)),
            "threshold": round(float(threshold), 4),
            "max_score": round(float(max_score), 4),
            "goals": seq
        }
        return json.dumps(data, ensure_ascii=False)

    def publish_ordered_goal_markers(self, ordered_goals, threshold, max_score):
        """
        RViz 顺序可视化：
        1. SPHERE：每个有序 goal 点；
        2. TEXT_VIEW_FACING：显示 01、02、03...，可选显示分数；
        3. ARROW：显示从上一个 goal 指向下一个 goal 的方向；
        4. LINE_STRIP：把所有 goal 串成线。

        注意：/ordered_goal_array 的 PoseArray 只能看点，不能显示编号；
        具体先后顺序要在 RViz 中添加 MarkerArray -> /ordered_goal_markers。
        """
        markers = MarkerArray()
        stamp = rospy.Time.now()

        # 先清空上一次 marker，避免路径点减少后残留旧编号。
        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        if not ordered_goals:
            self.pub_ordered_goal_markers.publish(markers)
            return

        lifetime_sec = float(self.ordered_goal_marker_lifetime)
        lifetime = rospy.Duration(lifetime_sec) if lifetime_sec > 0.0 else rospy.Duration(0.0)
        denom = max(float(max_score), 1e-6)

        # 构造用于画线/箭头的完整可视化航线：START -> ENTRY -> 01 -> 02 ...
        route_points = self.build_ordered_visual_route_points(
            ordered_goals,
            include_current=self.ordered_goal_include_current_position
        )

        # START 标记，方便看从哪里开始访问。
        if self.ordered_goal_show_start_marker and self.has_odom:
            start_sphere = Marker()
            start_sphere.header.stamp = stamp
            start_sphere.header.frame_id = self.frame_id
            start_sphere.ns = "ordered_goal_start"
            start_sphere.id = 0
            start_sphere.type = Marker.SPHERE
            start_sphere.action = Marker.ADD
            start_sphere.pose.position.x = float(self.current_x)
            start_sphere.pose.position.y = float(self.current_y)
            start_sphere.pose.position.z = float(self.current_z)
            start_sphere.pose.orientation.w = 1.0
            start_sphere.scale.x = float(self.ordered_goal_point_scale) * 1.2
            start_sphere.scale.y = float(self.ordered_goal_point_scale) * 1.2
            start_sphere.scale.z = float(self.ordered_goal_point_scale) * 1.2
            start_sphere.color.r = 0.1
            start_sphere.color.g = 0.8
            start_sphere.color.b = 1.0
            start_sphere.color.a = 1.0
            start_sphere.lifetime = lifetime
            markers.markers.append(start_sphere)

            start_text = Marker()
            start_text.header.stamp = stamp
            start_text.header.frame_id = self.frame_id
            start_text.ns = "ordered_goal_start_text"
            start_text.id = 0
            start_text.type = Marker.TEXT_VIEW_FACING
            start_text.action = Marker.ADD
            start_text.pose.position.x = float(self.current_x)
            start_text.pose.position.y = float(self.current_y)
            start_text.pose.position.z = float(self.current_z) + float(self.ordered_goal_text_z_offset)
            start_text.pose.orientation.w = 1.0
            start_text.scale.z = float(self.ordered_goal_text_scale) * 0.8
            start_text.color.r = 0.1
            start_text.color.g = 0.9
            start_text.color.b = 1.0
            start_text.color.a = 1.0
            start_text.text = "START"
            start_text.lifetime = lifetime
            markers.markers.append(start_text)

        # ENTRY 标记：显示从当前位置进入任务区域的边界交点。
        for x, y, z, _score, label, kind in route_points:
            if kind != "entry":
                continue

            entry_cube = Marker()
            entry_cube.header.stamp = stamp
            entry_cube.header.frame_id = self.frame_id
            entry_cube.ns = "ordered_goal_entry"
            entry_cube.id = 0
            entry_cube.type = Marker.CUBE
            entry_cube.action = Marker.ADD
            entry_cube.pose.position.x = float(x)
            entry_cube.pose.position.y = float(y)
            entry_cube.pose.position.z = float(z)
            entry_cube.pose.orientation.w = 1.0
            entry_cube.scale.x = float(self.ordered_goal_point_scale) * 1.15
            entry_cube.scale.y = float(self.ordered_goal_point_scale) * 1.15
            entry_cube.scale.z = float(self.ordered_goal_point_scale) * 1.15
            entry_cube.color.r = 0.0
            entry_cube.color.g = 1.0
            entry_cube.color.b = 0.35
            entry_cube.color.a = 1.0
            entry_cube.lifetime = lifetime
            markers.markers.append(entry_cube)

            entry_text = Marker()
            entry_text.header.stamp = stamp
            entry_text.header.frame_id = self.frame_id
            entry_text.ns = "ordered_goal_entry_text"
            entry_text.id = 0
            entry_text.type = Marker.TEXT_VIEW_FACING
            entry_text.action = Marker.ADD
            entry_text.pose.position.x = float(x)
            entry_text.pose.position.y = float(y)
            entry_text.pose.position.z = float(z) + float(self.ordered_goal_text_z_offset)
            entry_text.pose.orientation.w = 1.0
            entry_text.scale.z = float(self.ordered_goal_text_scale) * 0.75
            entry_text.color.r = 0.0
            entry_text.color.g = 1.0
            entry_text.color.b = 0.35
            entry_text.color.a = 1.0
            entry_text.text = str(label)
            entry_text.lifetime = lifetime
            markers.markers.append(entry_text)

        # 连接线，显示有序 goal 阵列的整体访问路径。
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.frame_id
        line.ns = "ordered_goal_line"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = float(self.ordered_goal_line_width)
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 0.1
        line.color.a = 0.95
        line.lifetime = lifetime

        for x, y, z, _score, _label, _kind in route_points:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            line.points.append(pt)

        markers.markers.append(line)

        # 箭头，明确显示从第 i 个点飞向第 i+1 个点。
        if self.ordered_goal_arrow_enabled and len(route_points) >= 2:
            arrow_id = 0
            for j in range(len(route_points) - 1):
                x1, y1, z1, _s1, _l1, _k1 = route_points[j]
                x2, y2, z2, _s2, _l2, _k2 = route_points[j + 1]
                d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
                if d < 1e-3:
                    continue

                arrow = Marker()
                arrow.header.stamp = stamp
                arrow.header.frame_id = self.frame_id
                arrow.ns = "ordered_goal_arrows"
                arrow.id = arrow_id
                arrow_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale.x = float(self.ordered_goal_arrow_shaft)      # shaft diameter
                arrow.scale.y = float(self.ordered_goal_arrow_head)       # head diameter
                arrow.scale.z = float(self.ordered_goal_arrow_head_len)   # head length
                arrow.color.r = 1.0
                arrow.color.g = 0.65
                arrow.color.b = 0.0
                arrow.color.a = 0.95
                arrow.lifetime = lifetime

                p1 = Point()
                p1.x = x1
                p1.y = y1
                p1.z = z1 + 0.08
                p2 = Point()
                p2.x = x2
                p2.y = y2
                p2.z = z2 + 0.08
                arrow.points.append(p1)
                arrow.points.append(p2)
                markers.markers.append(arrow)

        # 每个 goal 的球体和编号文字。
        for i, (x, y, z, score) in enumerate(ordered_goals, start=1):
            rel = float(np.clip(score / denom, 0.0, 1.0))

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self.frame_id
            sphere.ns = "ordered_goal_points"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = float(z)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = float(self.ordered_goal_point_scale)
            sphere.scale.y = float(self.ordered_goal_point_scale)
            sphere.scale.z = float(self.ordered_goal_point_scale)
            # 分数越高越偏红，低一些偏绿，避免和蓝色低分点混在一起。
            sphere.color.r = max(0.2, rel)
            sphere.color.g = max(0.1, 1.0 - rel)
            sphere.color.b = 0.05
            sphere.color.a = 1.0
            sphere.lifetime = lifetime
            markers.markers.append(sphere)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self.frame_id
            text.ns = "ordered_goal_numbers"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(x)
            text.pose.position.y = float(y)
            text.pose.position.z = float(z) + float(self.ordered_goal_text_z_offset)
            text.pose.orientation.w = 1.0
            text.scale.z = float(self.ordered_goal_text_scale)
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.1
            text.color.a = 1.0

            if self.ordered_goal_show_score:
                text.text = "%02d\n%.2f" % (i, float(score))
            else:
                text.text = "%02d" % i

            text.lifetime = lifetime
            markers.markers.append(text)

        self.pub_ordered_goal_markers.publish(markers)

    def publish_ordered_goal_array_outputs(self, ordered_goals, threshold, max_score, queue_action=None, queue_value=None):
        # 明确的有序 goal 阵列。数组下标就是执行顺序。
        ordered_array = self.make_pose_array_msg(
            ordered_goals,
            include_current=self.ordered_goal_array_include_current_position
        )
        self.pub_ordered_goal_array.publish(ordered_array)

        self.publish_ordered_goal_path(ordered_goals)
        self.publish_ordered_goal_markers(ordered_goals, threshold, max_score)

        sequence_json = self.make_ordered_goal_sequence_json(
            ordered_goals,
            threshold,
            max_score,
            queue_action=queue_action,
            queue_value=queue_value
        )
        self.pub_ordered_goal_sequence.publish(String(data=sequence_json))

    def ordered_goal_sequence_value(self, ordered_goals):
        """
        计算一个有序 goal 队列的整体价值。
        前面的 goal 权重大，后面的 goal 权重按 discount 递减。
        这样可以让队列更关注前几个即将访问的目标点，而不是只看总分。
        """
        if not ordered_goals:
            return 0.0

        discount = float(np.clip(self.ordered_goal_sequence_discount, 0.0, 1.0))
        value = 0.0
        weight = 1.0

        for _x, _y, _z, score in ordered_goals:
            value += weight * float(score)
            weight *= discount

        return float(value)

    def drop_reached_ordered_goals(self, ordered_goals):
        """
        自动到达删除。
        只检查队首 goal，并要求在半径内连续保持 ordered_goal_reached_hold_time 秒后才 pop。
        默认关闭；更推荐由控制器发布 /ordered_goal_reached 来显式删除队首。
        """
        if not ordered_goals:
            self.ordered_goal_auto_reached_since = 0.0
            return [], 0

        if not self.ordered_goal_drop_reached or not self.has_odom:
            self.ordered_goal_auto_reached_since = 0.0
            return list(ordered_goals), 0

        first = ordered_goals[0]
        dx = float(self.current_x) - float(first[0])
        dy = float(self.current_y) - float(first[1])
        dz = abs(float(self.current_z) - float(first[2]))

        xy_dist = math.hypot(dx, dy)
        xy_radius = max(0.0, float(self.ordered_goal_reached_xy_radius))
        z_radius = max(0.0, float(self.ordered_goal_reached_z_radius))
        inside = (xy_dist <= xy_radius) and (dz <= z_radius)

        now = time.time()
        if not inside:
            self.ordered_goal_auto_reached_since = 0.0
            return list(ordered_goals), 0

        if self.ordered_goal_auto_reached_since <= 0.0:
            self.ordered_goal_auto_reached_since = now
            return list(ordered_goals), 0

        hold_time = max(0.0, float(self.ordered_goal_reached_hold_time))
        if now - self.ordered_goal_auto_reached_since < hold_time:
            return list(ordered_goals), 0

        remaining = list(ordered_goals[1:])
        self.ordered_goal_auto_reached_since = 0.0
        return remaining, 1

    def should_replan_ordered_goal_queue(self, old_goals, new_goals, old_value, new_value, elapsed):
        """
        判断是否用实时生成的新队列替换稳定队列。

        严格锁存模式 ordered_goal_strict_lock=True 时：
        - 旧队列非空：默认永远保持旧队列；
        - 只有外部 /ordered_goal_reached 删除队首、/ordered_goal_reset 手动重置、队列为空后重新装载，才改变顺序；
        - 可选 ordered_goal_allow_force_replan_in_lock=True 时才允许超时强制重规划。
        """
        if not self.ordered_goal_queue_enable and not self.routes_manual_refresh:
            return True, "realtime"

        if not old_goals:
            if new_goals:
                return True, "init_queue"
            return bool(self.ordered_goal_allow_empty_replan), "empty_init"

        if not new_goals:
            if self.ordered_goal_allow_empty_replan:
                return True, "clear_empty"
            return False, "keep_new_empty"

        min_interval = max(0.0, float(self.ordered_goal_min_replan_interval))
        force_interval = max(min_interval, float(self.ordered_goal_force_replan_interval))

        if self.ordered_goal_strict_lock:
            if self.ordered_goal_allow_force_replan_in_lock and elapsed >= force_interval:
                return True, "strict_force_replan"
            return False, "strict_keep_locked"

        if elapsed < min_interval:
            return False, "keep_min_interval"

        if elapsed >= force_interval:
            return True, "force_replan"

        ratio = max(1.0, float(self.ordered_goal_replan_score_ratio))
        if new_value > old_value * ratio:
            return True, "better_score_replan"

        return False, "keep_not_better"

    def update_stable_ordered_goal_queue(self, new_ordered_goals, threshold, max_score):
        """
        把实时生成的 ordered_goals 转换为稳定任务队列。
        返回：stable_goals, stable_threshold, stable_max_score, action, stable_value
        """
        now = time.time()
        new_ordered_goals = list(new_ordered_goals)
        new_value = self.ordered_goal_sequence_value(new_ordered_goals)

        with self.queue_lock:
            if not self.ordered_goal_queue_enable and not self.routes_manual_refresh:
                self.stable_ordered_goals = list(new_ordered_goals)
                self.stable_ordered_goal_value = new_value
                self.stable_ordered_goal_time = now
                self.stable_ordered_goal_threshold = float(threshold)
                self.stable_ordered_goal_max_score = float(max_score)
                self.stable_ordered_goal_last_action = "realtime"
                return list(new_ordered_goals), float(threshold), float(max_score), "realtime", new_value

            # 手动重置优先级最高：清空稳定队列，当前周期会重新装载 new_ordered_goals。
            if self.ordered_goal_reset_requested:
                self.ordered_goal_reset_requested = False
                self.stable_ordered_goals = []
                self.stable_ordered_goal_value = 0.0
                self.stable_ordered_goal_time = 0.0
                self.stable_ordered_goal_threshold = 0.0
                self.stable_ordered_goal_max_score = 0.0
                self.stable_ordered_goal_last_action = "manual_reset"
                self.ordered_goal_auto_reached_since = 0.0

            action_prefix = None

            # 外部控制器确认到达：只删除队首一个 goal，不重排剩余队列。
            if self.ordered_goal_external_reached_requested:
                self.ordered_goal_external_reached_requested = False
                if self.stable_ordered_goals:
                    self.stable_ordered_goals = list(self.stable_ordered_goals[1:])
                    self.stable_ordered_goal_value = self.ordered_goal_sequence_value(self.stable_ordered_goals)
                    self.stable_ordered_goal_last_action = "external_reached_pop"
                    action_prefix = "external_reached_pop"
                    self.ordered_goal_auto_reached_since = 0.0

            # 可选自动到达删除。默认关闭；开启后也只删除队首一个，并需要连续保持。
            old_goals, dropped = self.drop_reached_ordered_goals(self.stable_ordered_goals)
            if dropped > 0:
                self.stable_ordered_goals = list(old_goals)
                self.stable_ordered_goal_value = self.ordered_goal_sequence_value(old_goals)
                self.stable_ordered_goal_last_action = "auto_drop_reached_%d" % dropped
                action_prefix = "auto_drop_reached_%d" % dropped

            old_value = self.ordered_goal_sequence_value(self.stable_ordered_goals)
            elapsed = 1e9 if self.stable_ordered_goal_time <= 0.0 else now - self.stable_ordered_goal_time

            do_replan, action = self.should_replan_ordered_goal_queue(
                self.stable_ordered_goals,
                new_ordered_goals,
                old_value,
                new_value,
                elapsed
            )

            # 队列为空时允许重新装载新队列。
            # 这包括第一次运行、全部 goal 已执行完、手动 reset 后重新生成。
            if not self.stable_ordered_goals and new_ordered_goals:
                do_replan = True
                if action_prefix:
                    action = action_prefix + "_reload"
                elif self.stable_ordered_goal_last_action == "manual_reset":
                    action = "manual_reset_reload"
                else:
                    action = "init_queue"

            if do_replan:
                self.stable_ordered_goals = list(new_ordered_goals)
                self.stable_ordered_goal_value = new_value
                self.stable_ordered_goal_time = now
                self.stable_ordered_goal_threshold = float(threshold)
                self.stable_ordered_goal_max_score = float(max_score)
                self.stable_ordered_goal_last_action = action
            else:
                if action_prefix is not None:
                    action = action_prefix + "_keep"
                self.stable_ordered_goal_last_action = action

            return (
                list(self.stable_ordered_goals),
                float(self.stable_ordered_goal_threshold),
                float(self.stable_ordered_goal_max_score),
                str(self.stable_ordered_goal_last_action),
                float(self.stable_ordered_goal_value)
            )

    def goal_tuple_to_point_dict(self, goal):
        return {
            "x": round(float(goal[0]), 3),
            "y": round(float(goal[1]), 3),
            "z": round(float(goal[2]), 3),
            "score": round(float(goal[3]), 4) if len(goal) >= 4 else 0.0
        }

    def point_segment_distance3(self, p, a, b):
        px, py, pz = float(p[0]), float(p[1]), float(p[2])
        ax, ay, az = float(a[0]), float(a[1]), float(a[2])
        bx, by, bz = float(b[0]), float(b[1]), float(b[2])

        abx = bx - ax
        aby = by - ay
        abz = bz - az
        apx = px - ax
        apy = py - ay
        apz = pz - az
        denom = abx * abx + aby * aby + abz * abz

        if denom < 1e-9:
            return math.sqrt(apx * apx + apy * apy + apz * apz)

        t = (apx * abx + apy * aby + apz * abz) / denom
        t = max(0.0, min(1.0, t))
        cx = ax + t * abx
        cy = ay + t * aby
        cz = az + t * abz
        dx = px - cx
        dy = py - cy
        dz = pz - cz
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def min_distance_to_main_route(self, point, main_goals):
        if not main_goals:
            return 1e9

        p = (float(point[0]), float(point[1]), float(point[2]))
        main_pts = [g[:3] for g in main_goals]

        best = min(self.dist3(p, q) for q in main_pts)
        if len(main_pts) >= 2:
            for i in range(len(main_pts) - 1):
                best = min(best, self.point_segment_distance3(p, main_pts[i], main_pts[i + 1]))

        return float(best)

    def nearest_main_goal_index(self, point, main_goals):
        if not main_goals:
            return None, 1e9

        best_i = 0
        best_d = None
        for i, g in enumerate(main_goals):
            d = self.dist3(point, g[:3])
            if best_d is None or d < best_d:
                best_d = d
                best_i = i

        return best_i, float(best_d)

    def main_direction_at_attach(self, main_goals, attach_idx):
        """
        返回主线路在分叉点处的前进方向。
        优先使用 attach_idx -> attach_idx+1；末端用 attach_idx-1 -> attach_idx；
        如果主线路只有一个点，则使用当前位置 -> 第一个点。
        """
        if not main_goals:
            return None

        if len(main_goals) >= 2:
            if attach_idx < len(main_goals) - 1:
                a = main_goals[attach_idx][:3]
                b = main_goals[attach_idx + 1][:3]
            else:
                a = main_goals[attach_idx - 1][:3]
                b = main_goals[attach_idx][:3]

            return (
                float(b[0]) - float(a[0]),
                float(b[1]) - float(a[1]),
                float(b[2]) - float(a[2])
            )

        if self.has_odom:
            b = main_goals[0][:3]
            return (
                float(b[0]) - float(self.current_x),
                float(b[1]) - float(self.current_y),
                float(b[2]) - float(self.current_z)
            )

        return None

    def angle_between_vectors_deg(self, v1, v2):
        if v1 is None or v2 is None:
            return 180.0

        if self.branch_use_xy_angle:
            a = np.asarray([float(v1[0]), float(v1[1])], dtype=np.float32)
            b = np.asarray([float(v2[0]), float(v2[1])], dtype=np.float32)
        else:
            a = np.asarray([float(v1[0]), float(v1[1]), float(v1[2])], dtype=np.float32)
            b = np.asarray([float(v2[0]), float(v2[1]), float(v2[2])], dtype=np.float32)

        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na < 1e-6 or nb < 1e-6:
            return 180.0

        c = float(np.dot(a, b) / max(na * nb, 1e-6))
        c = float(np.clip(c, -1.0, 1.0))
        return math.degrees(math.acos(c))

    def branch_candidate_value(self, score, angle_deg, attach_dist, route_dist):
        angle_gain = 0.5 + min(max(float(angle_deg), 0.0), 180.0) / 180.0
        dist_penalty = 1.0 + 0.12 * float(attach_dist) + 0.08 * float(route_dist)
        return float(score) * angle_gain / dist_penalty

    def collect_branch_candidates(self, candidates, scores, main_goals, max_score):
        if scores.size == 0 or not main_goals:
            return []

        branch_min_score = max(
            float(self.branch_min_score),
            float(max_score) * float(np.clip(self.branch_score_ratio, 0.0, 1.0))
        )

        order = np.argsort(scores)[::-1]
        pool_n = min(max(1, int(self.branch_candidate_pool)), len(order))
        infos = []

        cur_pos = None
        if self.has_odom:
            cur_pos = (float(self.current_x), float(self.current_y), float(self.current_z))

        for idx in order[:pool_n]:
            score = float(scores[idx])
            if score < branch_min_score:
                # order 已经按分数降序排列，后续点更低，直接退出。
                break

            x, y, z = candidates[idx]
            goal = (float(x), float(y), float(z), score)
            point = goal[:3]

            if cur_pos is not None and self.dist3(cur_pos, point) < float(self.min_goal_distance):
                continue

            attach_idx, attach_dist = self.nearest_main_goal_index(point, main_goals)
            if attach_idx is None:
                continue

            force_seq = int(self.branch_force_attach_seq)
            if force_seq > 0 and int(attach_idx) != force_seq - 1:
                continue

            if attach_dist > float(self.branch_attach_radius):
                continue

            route_dist = self.min_distance_to_main_route(point, main_goals)
            if route_dist < float(self.branch_min_off_main_dist):
                continue

            attach_goal = main_goals[attach_idx]
            main_dir = self.main_direction_at_attach(main_goals, attach_idx)
            branch_dir = (
                float(point[0]) - float(attach_goal[0]),
                float(point[1]) - float(attach_goal[1]),
                float(point[2]) - float(attach_goal[2])
            )
            angle_deg = self.angle_between_vectors_deg(main_dir, branch_dir)
            if angle_deg < float(self.branch_min_angle_deg):
                continue

            infos.append({
                "idx": int(idx),
                "goal": goal,
                "score": float(score),
                "attach_idx": int(attach_idx),
                "attach_dist": float(attach_dist),
                "route_dist": float(route_dist),
                "angle_deg": float(angle_deg),
                "value": self.branch_candidate_value(score, angle_deg, attach_dist, route_dist)
            })

        infos.sort(key=lambda item: item["value"], reverse=True)
        return infos

    def route_point_too_close(self, goal, route_goals, min_sep):
        for g in route_goals:
            if self.dist3(goal[:3], g[:3]) < min_sep:
                return True
        return False

    def extend_branch_route(self, seed_info, infos, used_global_points, main_goals):
        route = [seed_info["goal"]]
        used_indices = {seed_info["idx"]}
        attach_idx = int(seed_info["attach_idx"])
        attach_goal = main_goals[attach_idx]
        min_sep = max(0.0, float(self.branch_min_separation))
        max_len = max(1, int(self.branch_max_len))

        while len(route) < max_len:
            current = route[-1]
            current_attach_dist = self.dist3(current[:3], attach_goal[:3])
            best = None
            best_value = -1.0

            for info in infos:
                if info["idx"] in used_indices:
                    continue

                if int(info["attach_idx"]) != attach_idx:
                    continue

                goal = info["goal"]
                if self.route_point_too_close(goal, route, min_sep):
                    continue

                if self.route_point_too_close(goal, used_global_points, min_sep):
                    continue

                d_cur = self.dist3(current[:3], goal[:3])
                if d_cur > float(self.branch_extend_radius):
                    continue

                # 优先向远离分叉点的方向推进，允许轻微回退但不允许原地绕圈。
                goal_attach_dist = self.dist3(goal[:3], attach_goal[:3])
                if goal_attach_dist + 0.25 < current_attach_dist:
                    continue

                value = float(info["score"]) / (1.0 + 0.25 * d_cur)
                value += 0.15 * float(info["value"])
                if value > best_value:
                    best_value = value
                    best = info

            if best is None:
                break

            route.append(best["goal"])
            used_indices.add(best["idx"])

        return route

    def branch_route_value(self, route):
        discount = float(np.clip(self.branch_route_discount, 0.0, 1.0))
        value = 0.0
        weight = 1.0
        length_penalty = 0.0

        prev = None
        for goal in route:
            value += weight * float(goal[3])
            if prev is not None:
                length_penalty += 0.03 * self.dist3(prev[:3], goal[:3])
            prev = goal
            weight *= discount

        return float(max(0.0, value - length_penalty))

    def build_backup_routes(self, candidates, scores, main_goals, threshold, max_score):
        """
        从主线路的分叉点附近生成备用探索线路。备用线路不改变主队列，
        只通过 /backup_goal_markers 和 /backup_goal_sequence 输出；
        如需手动切换，可向 /select_backup_route 发布 B1/B2/...。
        """
        if not self.branch_enable:
            return []

        if not main_goals:
            return []

        infos = self.collect_branch_candidates(candidates, scores, main_goals, max_score)
        if not infos:
            return []

        max_routes = max(0, int(self.branch_max_routes))
        if max_routes <= 0:
            return []

        routes = []
        used_global_points = []
        attach_counts = {}
        min_sep = max(0.0, float(self.branch_min_separation))
        max_routes_per_attach = max(1, int(self.branch_max_routes_per_attach))

        for seed in infos:
            if len(routes) >= max_routes:
                break

            attach_idx = int(seed["attach_idx"])
            if attach_counts.get(attach_idx, 0) >= max_routes_per_attach:
                continue

            if self.route_point_too_close(seed["goal"], used_global_points, min_sep):
                continue

            route_goals = self.extend_branch_route(seed, infos, used_global_points, main_goals)
            if not route_goals:
                continue

            # 严格挂载检查：备用路线必须从某个主线路节点分叉。
            # 如果第一个支线点离挂载节点太远，就丢弃这条路线，避免出现“孤立备用线路”。
            if self.branch_require_attached:
                attach_goal = main_goals[attach_idx]
                first_dist = self.dist3(route_goals[0][:3], attach_goal[:3])
                if first_dist > float(self.branch_max_first_branch_dist):
                    continue

            for g in route_goals:
                used_global_points.append(g)

            route_id = "B%d" % (len(routes) + 1)
            value = self.branch_route_value(route_goals)
            attach_counts[attach_idx] = attach_counts.get(attach_idx, 0) + 1

            routes.append({
                "route_id": route_id,
                "branch_from_seq": int(attach_idx + 1),
                "branch_from_goal": main_goals[attach_idx],
                # 保存主线路前缀。这样 B1 可以表示为 01 -> 02 -> 03 -> B1-01 -> B1-02，
                # 而不是只表示 03 -> B1-01 -> B1-02。
                "main_prefix_goals": list(main_goals[:attach_idx + 1]),
                # branch-only 支线点。完整备用路线由 build_backup_full_route_goals() 生成。
                "branch_goals": list(route_goals),
                "goals": list(route_goals),
                "value": float(value),
                "seed_score": float(seed["score"]),
                "seed_angle_deg": float(seed["angle_deg"]),
                "seed_attach_dist": float(seed["attach_dist"]),
                "threshold": float(max(float(self.branch_min_score), float(max_score) * float(self.branch_score_ratio)))
            })

        return routes

    def build_backup_full_route_goals(self, route, main_goals=None):
        """
        构造完整备用路线。

        默认输出：主线路前缀 + 分叉支线，例如：
        01 -> 02 -> 03 -> B1-01 -> B1-02 -> B1-03。

        如果 ~branch_include_main_prefix:=false，则退化为旧逻辑：
        03 -> B1-01 -> B1-02，或者只显示 B1-01 -> B1-02。
        """
        if main_goals is None:
            main_goals = []

        branch_goals = list(route.get("branch_goals", route.get("goals", [])))
        full_goals = []

        if self.branch_include_main_prefix:
            prefix_goals = list(route.get("main_prefix_goals", []))

            # 兼容旧 route：如果没有保存 main_prefix_goals，则用当前 main_goals 按 branch_from_seq 重建。
            if not prefix_goals and main_goals:
                attach_idx = max(0, int(route.get("branch_from_seq", 1)) - 1)
                prefix_goals = list(main_goals[:attach_idx + 1])

            if prefix_goals:
                full_goals.extend(prefix_goals)
            elif self.branch_include_attach_point and "branch_from_goal" in route:
                full_goals.append(route["branch_from_goal"])
        else:
            if self.branch_include_attach_point and "branch_from_goal" in route:
                full_goals.append(route["branch_from_goal"])

        full_goals.extend(branch_goals)
        return full_goals

    def make_backup_goal_sequence_json(self, backup_routes, main_goals, threshold, max_score, backup_action=None, cache_age=None):
        routes_data = []
        for route in backup_routes:
            route_id = str(route["route_id"])
            branch_goals = list(route.get("branch_goals", route.get("goals", [])))
            full_route_goals = self.build_backup_full_route_goals(route, main_goals)

            main_prefix_data = []
            for i, g in enumerate(route.get("main_prefix_goals", []), start=1):
                main_prefix_data.append({
                    "seq": "%02d" % int(i),
                    "x": round(float(g[0]), 3),
                    "y": round(float(g[1]), 3),
                    "z": round(float(g[2]), 3),
                    "score": round(float(g[3]), 4)
                })

            branch_data = []
            for i, g in enumerate(branch_goals, start=1):
                branch_data.append({
                    "seq": "%s-%02d" % (route_id, i),
                    "x": round(float(g[0]), 3),
                    "y": round(float(g[1]), 3),
                    "z": round(float(g[2]), 3),
                    "score": round(float(g[3]), 4)
                })

            full_data = []
            prefix_len = len(full_route_goals) - len(branch_goals)
            for i, g in enumerate(full_route_goals):
                if i < prefix_len:
                    label = "%02d" % int(i + 1)
                else:
                    label = "%s-%02d" % (route_id, int(i - prefix_len + 1))

                full_data.append({
                    "seq": label,
                    "x": round(float(g[0]), 3),
                    "y": round(float(g[1]), 3),
                    "z": round(float(g[2]), 3),
                    "score": round(float(g[3]), 4)
                })

            first_branch_dist = None
            if branch_goals:
                first_branch_dist = self.dist3(branch_goals[0][:3], route["branch_from_goal"][:3])

            routes_data.append({
                "route_id": route_id,
                "route_mode": "main_prefix_then_branch" if self.branch_include_main_prefix else "branch_only",
                "attached": bool(route.get("branch_from_goal", None) is not None and len(branch_goals) > 0),
                "branch_from_seq": int(route["branch_from_seq"]),
                "branch_from": self.goal_tuple_to_point_dict(route["branch_from_goal"]),
                "first_branch_dist": None if first_branch_dist is None else round(float(first_branch_dist), 3),
                "value": round(float(route["value"]), 4),
                "seed_score": round(float(route["seed_score"]), 4),
                "seed_angle_deg": round(float(route["seed_angle_deg"]), 1),
                # main_prefix 是 01 -> 02 -> 03 这段。
                "main_prefix": main_prefix_data,
                # branch_goals 是 B1-01 -> B1-02 这段。
                "branch_goals": branch_data,
                # goals 是完整备用路线：01 -> 02 -> 03 -> B1-01 -> B1-02。
                "goals": full_data
            })

        data = {
            "event": "backup_goal_sequence",
            "frame_id": self.frame_id,
            "stamp": rospy.Time.now().to_sec(),
            "branch_enable": bool(self.branch_enable),
            "manual_refresh": bool(self.backup_routes_manual_refresh),
            "backup_action": str(backup_action if backup_action is not None else self.cached_backup_action),
            "cache_age": round(float(cache_age if cache_age is not None else (0.0 if self.cached_backup_time <= 0.0 else max(0.0, time.time() - self.cached_backup_time))), 2),
            "backup_refresh_topic": self.backup_routes_refresh_topic,
            "main_refresh_topic": self.main_routes_refresh_topic,
            "combined_refresh_topic": self.routes_refresh_topic,
            "route_mode": "main_prefix_then_branch" if self.branch_include_main_prefix else "branch_only",
            "num_main_goals": int(len(main_goals)),
            "num_routes": int(len(backup_routes)),
            "threshold": round(float(threshold), 4),
            "max_score": round(float(max_score), 4),
            "routes": routes_data
        }
        return json.dumps(data, ensure_ascii=False)

    def backup_route_color(self, route_index):
        # 返回 RGB，取值范围 0~1。不同备用线路用不同颜色，与主线路黄色区分。
        palette = [
            (0.1, 0.9, 1.0),
            (1.0, 0.35, 1.0),
            (0.2, 1.0, 0.45),
            (1.0, 0.55, 0.15),
            (0.55, 0.75, 1.0)
        ]
        return palette[int(route_index) % len(palette)]

    def publish_backup_goal_markers(self, backup_routes):
        markers = MarkerArray()
        stamp = rospy.Time.now()

        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = self.frame_id
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        lifetime_sec = float(self.backup_goal_marker_lifetime)
        lifetime = rospy.Duration(lifetime_sec) if lifetime_sec > 0.0 else rospy.Duration(0.0)

        marker_id = 0
        for r_idx, route in enumerate(backup_routes):
            route_id = str(route["route_id"])
            cr, cg, cb = self.backup_route_color(r_idx)
            branch_from = route.get("branch_from_goal", None)
            goals = list(route.get("branch_goals", route.get("goals", [])))
            if branch_from is None or (self.branch_require_attached and not goals):
                continue

            full_route_goals = self.build_backup_full_route_goals(route)
            if self.branch_require_attached and len(full_route_goals) < 2:
                continue

            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self.frame_id
            line.ns = "backup_%s_line" % route_id
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = float(self.backup_goal_line_width)
            line.color.r = cr
            line.color.g = cg
            line.color.b = cb
            line.color.a = 0.95
            line.lifetime = lifetime

            # 线段显示完整备用路线。默认是：01 -> 02 -> 03 -> B1-01 -> B1-02。
            # 如果关闭 branch_include_main_prefix，则显示旧版：03 -> B1-01 -> B1-02。
            line_goals = full_route_goals if self.branch_draw_main_prefix else []
            if not line_goals:
                if self.branch_include_attach_point:
                    line_goals = [branch_from] + goals
                else:
                    line_goals = list(goals)

            for g in line_goals:
                pt = Point()
                pt.x = float(g[0])
                pt.y = float(g[1])
                pt.z = float(g[2]) + 0.05
                line.points.append(pt)

            markers.markers.append(line)

            # 分叉点实体标记：把备用线路真正挂载到主线路节点上。
            if self.branch_draw_attach_sphere:
                attach_sphere = Marker()
                attach_sphere.header.stamp = stamp
                attach_sphere.header.frame_id = self.frame_id
                attach_sphere.ns = "backup_%s_attach_point" % route_id
                attach_sphere.id = marker_id
                marker_id += 1
                attach_sphere.type = Marker.CUBE
                attach_sphere.action = Marker.ADD
                attach_sphere.pose.position.x = float(branch_from[0])
                attach_sphere.pose.position.y = float(branch_from[1])
                attach_sphere.pose.position.z = float(branch_from[2])
                attach_sphere.pose.orientation.w = 1.0
                attach_sphere.scale.x = float(self.backup_goal_point_scale) * 1.35
                attach_sphere.scale.y = float(self.backup_goal_point_scale) * 1.35
                attach_sphere.scale.z = float(self.backup_goal_point_scale) * 1.35
                attach_sphere.color.r = cr
                attach_sphere.color.g = cg
                attach_sphere.color.b = cb
                attach_sphere.color.a = 1.0
                attach_sphere.lifetime = lifetime
                markers.markers.append(attach_sphere)

            # 分叉点标记：显示 Bx@主线序号
            attach_text = Marker()
            attach_text.header.stamp = stamp
            attach_text.header.frame_id = self.frame_id
            attach_text.ns = "backup_%s_attach_text" % route_id
            attach_text.id = marker_id
            marker_id += 1
            attach_text.type = Marker.TEXT_VIEW_FACING
            attach_text.action = Marker.ADD
            attach_text.pose.position.x = float(branch_from[0])
            attach_text.pose.position.y = float(branch_from[1])
            attach_text.pose.position.z = float(branch_from[2]) + float(self.backup_goal_text_z_offset) * 1.25
            attach_text.pose.orientation.w = 1.0
            attach_text.scale.z = float(self.backup_goal_text_scale) * 0.85
            attach_text.color.r = cr
            attach_text.color.g = cg
            attach_text.color.b = cb
            attach_text.color.a = 1.0
            attach_text.text = "%s@%02d" % (route_id, int(route["branch_from_seq"]))
            attach_text.lifetime = lifetime
            markers.markers.append(attach_text)

            for i, g in enumerate(goals, start=1):
                sphere = Marker()
                sphere.header.stamp = stamp
                sphere.header.frame_id = self.frame_id
                sphere.ns = "backup_%s_points" % route_id
                sphere.id = marker_id
                marker_id += 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = float(g[0])
                sphere.pose.position.y = float(g[1])
                sphere.pose.position.z = float(g[2])
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = float(self.backup_goal_point_scale)
                sphere.scale.y = float(self.backup_goal_point_scale)
                sphere.scale.z = float(self.backup_goal_point_scale)
                sphere.color.r = cr
                sphere.color.g = cg
                sphere.color.b = cb
                sphere.color.a = 1.0
                sphere.lifetime = lifetime
                markers.markers.append(sphere)

                text = Marker()
                text.header.stamp = stamp
                text.header.frame_id = self.frame_id
                text.ns = "backup_%s_text" % route_id
                text.id = marker_id
                marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = float(g[0])
                text.pose.position.y = float(g[1])
                text.pose.position.z = float(g[2]) + float(self.backup_goal_text_z_offset)
                text.pose.orientation.w = 1.0
                text.scale.z = float(self.backup_goal_text_scale)
                text.color.r = cr
                text.color.g = cg
                text.color.b = cb
                text.color.a = 1.0
                text.text = "%s-%02d\n%.2f" % (route_id, i, float(g[3]))
                text.lifetime = lifetime
                markers.markers.append(text)

        self.pub_backup_goal_markers.publish(markers)

    def get_or_update_backup_routes(self, candidates, scores, main_goals, threshold, max_score):
        """
        获取备用线路。
        backup_routes_manual_refresh=True 时，备用线路只在以下情况重建：
        1. 节点第一次运行，尚无缓存；
        2. 收到 /backup_routes_refresh；
        3. 收到兼容联合刷新 /search_routes_refresh。

        /main_routes_refresh 和 /ordered_goal_reset 只刷新主线路，不会自动重建备用线路。
        其余发布周期只重复发布缓存的备用线路，避免 /backup_goal_markers 每秒重刷、跳变。
        """
        if not self.branch_enable:
            with self.queue_lock:
                self.cached_backup_routes = []
                self.cached_backup_valid = True
                self.cached_backup_main_goals = list(main_goals)
                self.cached_backup_threshold = float(threshold)
                self.cached_backup_max_score = float(max_score)
                self.cached_backup_time = time.time()
                self.cached_backup_action = "branch_disabled"
            return [], float(threshold), float(max_score), "branch_disabled", 0.0

        if not self.backup_routes_manual_refresh:
            routes = self.build_backup_routes(candidates, scores, main_goals, threshold, max_score)
            now = time.time()
            with self.queue_lock:
                self.cached_backup_routes = list(routes)
                self.cached_backup_valid = True
                self.cached_backup_main_goals = list(main_goals)
                self.cached_backup_threshold = float(threshold)
                self.cached_backup_max_score = float(max_score)
                self.cached_backup_time = now
                self.cached_backup_action = "realtime"
            return list(routes), float(threshold), float(max_score), "realtime", 0.0

        with self.queue_lock:
            need_refresh = bool(self.backup_routes_refresh_requested) or (not self.cached_backup_valid)
            if need_refresh:
                self.backup_routes_refresh_requested = False
                requested_action = str(self.cached_backup_action)
            else:
                requested_action = "manual_keep"

        if need_refresh:
            routes = self.build_backup_routes(candidates, scores, main_goals, threshold, max_score)
            now = time.time()
            if requested_action in ("init", "manual_keep"):
                action = "manual_init" if not self.cached_backup_valid else "manual_refresh"
            else:
                action = requested_action.replace("_requested", "")
            if not routes:
                action = action + "_empty"

            with self.queue_lock:
                self.cached_backup_routes = list(routes)
                self.cached_backup_valid = True
                self.cached_backup_main_goals = list(main_goals)
                self.cached_backup_threshold = float(threshold)
                self.cached_backup_max_score = float(max_score)
                self.cached_backup_time = now
                self.cached_backup_action = action

            return list(routes), float(threshold), float(max_score), action, 0.0

        with self.queue_lock:
            routes = list(self.cached_backup_routes)
            cached_threshold = float(self.cached_backup_threshold)
            cached_max_score = float(self.cached_backup_max_score)
            cache_age = 0.0 if self.cached_backup_time <= 0.0 else max(0.0, time.time() - self.cached_backup_time)
            action = "manual_keep"

        return routes, cached_threshold, cached_max_score, action, cache_age

    def publish_backup_goal_outputs(self, backup_routes, main_goals, threshold, max_score, backup_action=None, cache_age=None):
        self.last_backup_routes = list(backup_routes)
        self.publish_backup_goal_markers(backup_routes)
        sequence_json = self.make_backup_goal_sequence_json(
            backup_routes,
            main_goals,
            threshold,
            max_score,
            backup_action=backup_action,
            cache_age=cache_age
        )
        self.pub_backup_goal_sequence.publish(String(data=sequence_json))

    def apply_selected_backup_route_if_requested(self, stable_goals, backup_routes, threshold, max_score):
        if not self.enable_backup_route_selection:
            return list(stable_goals), None, self.ordered_goal_sequence_value(stable_goals)

        with self.queue_lock:
            route_id = self.selected_backup_route_id
            if not route_id:
                return list(stable_goals), None, self.ordered_goal_sequence_value(stable_goals)

            selected = None
            for route in backup_routes:
                if str(route["route_id"]).upper() == str(route_id).upper():
                    selected = route
                    break

            if selected is None:
                rospy.logwarn_throttle(2.0, "Selected backup route %s not found in current backup routes.", str(route_id))
                return list(stable_goals), None, self.ordered_goal_sequence_value(stable_goals)

            # 手动切换到 B1/B2 时，执行队列采用完整备用路线。
            # 默认效果：01 -> 02 -> 03 -> B1-01 -> B1-02 -> B1-03。
            # 这样备用路线不是孤立支线，而是从主路线前缀自然延伸到分叉支线。
            new_queue = self.build_backup_full_route_goals(selected, stable_goals)

            new_value = self.ordered_goal_sequence_value(new_queue)
            self.stable_ordered_goals = list(new_queue)
            self.stable_ordered_goal_value = new_value
            self.stable_ordered_goal_time = time.time()
            self.stable_ordered_goal_threshold = float(threshold)
            self.stable_ordered_goal_max_score = float(max_score)
            self.stable_ordered_goal_last_action = "manual_select_%s" % str(route_id).upper()
            self.selected_backup_route_id = None
            self.ordered_goal_auto_reached_since = 0.0

            rospy.loginfo("Applied backup full route %s as main ordered queue, points=%d", str(route_id).upper(), len(new_queue))
            return list(new_queue), self.stable_ordered_goal_last_action, new_value

    def publish_high_score_goals_and_path(self, candidates, scores):
        goals, threshold, max_score = self.extract_high_score_goals(candidates, scores)
        realtime_ordered_goals = self.order_goals_for_path(goals)

        stable_ordered_goals, stable_threshold, stable_max_score, queue_action, queue_value = \
            self.update_stable_ordered_goal_queue(realtime_ordered_goals, threshold, max_score)

        # 备用分支线路基于当前稳定主线路生成。默认备用路线是“主线前缀 + 分叉支线”；
        # 例如 01 -> 02 -> 03 -> B1-01 -> B1-02。
        # 只有收到 /select_backup_route 的 B1/B2/... 指令时才切换主队列。
        backup_routes, backup_threshold, backup_max_score, backup_action, backup_cache_age = \
            self.get_or_update_backup_routes(
                candidates,
                scores,
                stable_ordered_goals,
                stable_threshold,
                stable_max_score
            )

        selected_goals, selected_action, selected_value = self.apply_selected_backup_route_if_requested(
            stable_ordered_goals,
            backup_routes,
            stable_threshold,
            stable_max_score
        )

        if selected_action is not None:
            stable_ordered_goals = selected_goals
            queue_action = selected_action
            queue_value = selected_value
            # 切换到备用线路后，不在手动刷新模式下自动重建备用线路。
            # 之后可用 /main_routes_refresh 只刷新主线，或用 /backup_routes_refresh 只刷新备线。
            if not self.backup_routes_manual_refresh:
                backup_routes, backup_threshold, backup_max_score, backup_action, backup_cache_age = \
                    self.get_or_update_backup_routes(
                        candidates,
                        scores,
                        stable_ordered_goals,
                        stable_threshold,
                        stable_max_score
                    )
            else:
                backup_action = selected_action + "_backup_cache_kept"

        # 1. 所有当前高分 goal 点：仍然实时输出，方便看最新热力图产生了哪些候选点。
        self.pub_high_goals.publish(self.make_pose_array_msg(goals))

        # 2. 稳定主路径点序列：用于实际执行和 RViz 顺序显示，不再每秒整体重排。
        self.pub_waypoints.publish(self.make_pose_array_msg(stable_ordered_goals))
        self.publish_waypoint_path(stable_ordered_goals)

        # 2.1 显式有序 goal 阵列：输出的是稳定主线路。
        self.publish_ordered_goal_array_outputs(
            stable_ordered_goals,
            stable_threshold,
            stable_max_score,
            queue_action=queue_action,
            queue_value=queue_value
        )

        # 2.2 备用探索线路：在主线路的分叉点处生成多条备选路线。
        self.publish_backup_goal_outputs(
            backup_routes,
            stable_ordered_goals,
            backup_threshold,
            backup_max_score,
            backup_action=backup_action,
            cache_age=backup_cache_age
        )

        # 3. 高分 goal 的彩色点云仍用当前实时 goals，便于观察热力图变化。
        self.publish_high_goal_cloud(goals, max_score)

        debug_goals = []
        for x, y, z, score in stable_ordered_goals[:min(len(stable_ordered_goals), 30)]:
            debug_goals.append({
                "x": round(float(x), 2),
                "y": round(float(y), 2),
                "z": round(float(z), 2),
                "score": round(float(score), 3)
            })

        debug_branches = []
        for route in backup_routes:
            debug_branches.append({
                "route_id": str(route["route_id"]),
                "from_seq": int(route["branch_from_seq"]),
                "len": int(len(route["goals"])),
                "value": round(float(route["value"]), 3),
                "seed_angle_deg": round(float(route["seed_angle_deg"]), 1)
            })

        debug = {
            "event": "stable_ordered_goal_queue",
            "raw_num_goals": int(len(goals)),
            "raw_num_path_points": int(len(realtime_ordered_goals)),
            "stable_num_path_points": int(len(stable_ordered_goals)),
            "backup_num_routes": int(len(backup_routes)),
            "backup_action": str(backup_action),
            "backup_cache_age": round(float(backup_cache_age), 2),
            "raw_threshold": round(float(threshold), 3),
            "stable_threshold": round(float(stable_threshold), 3),
            "raw_max_score": round(float(max_score), 3),
            "stable_max_score": round(float(stable_max_score), 3),
            "queue_value": round(float(queue_value), 3),
            "queue_action": str(queue_action),
            "strict_lock": bool(self.ordered_goal_strict_lock),
            "drop_reached_auto": bool(self.ordered_goal_drop_reached),
            "path_order_mode": str(self.path_order_mode),
            "goals": debug_goals,
            "backup_routes": debug_branches
        }
        self.pub_debug.publish(String(data=json.dumps(debug, ensure_ascii=False)))

        rospy.loginfo_throttle(
            1.0,
            "Stable ordered queue: raw_goals=%d raw_path=%d stable_path=%d backups=%d raw_max=%.3f stable_max=%.3f value=%.3f action=%s mode=%s strict=%s",
            len(goals),
            len(realtime_ordered_goals),
            len(stable_ordered_goals),
            len(backup_routes),
            max_score,
            stable_max_score,
            queue_value,
            str(queue_action),
            str(self.path_order_mode),
            str(self.ordered_goal_strict_lock)
        )

    def publish_next_goal(self, candidates, scores):
        if scores.size == 0:
            return

        max_score = float(np.max(scores))
        if max_score < self.min_goal_score:
            return

        now = time.time()
        order = np.argsort(scores)[::-1]

        # 1. 先取 Top-K 高分点，不直接使用单个最高分点。
        top_k = min(int(self.goal_cluster_top_k), len(order))
        if top_k <= 0:
            return

        top_idx = order[:top_k]

        # 2. 保留接近最高分的候选点，形成稳定高分团。
        score_threshold = max_score * float(np.clip(self.goal_cluster_ratio, 0.0, 1.0))
        cluster_idx = []

        for idx in top_idx:
            if float(scores[idx]) >= score_threshold:
                cluster_idx.append(int(idx))

        if not cluster_idx:
            cluster_idx = [int(order[0])]

        cluster_idx = np.asarray(cluster_idx, dtype=np.int32)

        # 3. 对高分团做加权中心，避免目标在多个相近格点之间跳变。
        pts = candidates[cluster_idx]
        ws = scores[cluster_idx].astype(np.float32)
        ws = np.maximum(ws, 1e-6)
        centroid = np.sum(pts * ws.reshape(-1, 1), axis=0) / np.sum(ws)

        # 4. 选择距离加权中心最近的实际候选点作为中心代表。
        d_to_centroid = np.linalg.norm(pts - centroid.reshape(1, 3), axis=1)
        center_idx = int(cluster_idx[int(np.argmin(d_to_centroid))])

        # 5. 二次筛选：原始分数 + 距离惩罚 + 跳变惩罚。
        best = None
        best_value = -1.0

        cur_pos = None
        if self.has_odom:
            cur_pos = (self.current_x, self.current_y, self.current_z)

        old_goal = self.current_goal
        candidate_indices = [center_idx] + [int(i) for i in top_idx]

        seen = set()
        for idx in candidate_indices:
            if idx in seen:
                continue
            seen.add(idx)

            raw_score = float(scores[idx])
            if raw_score < self.min_goal_score:
                continue

            x, y, z = candidates[idx]
            goal = (float(x), float(y), float(z))

            dist_cost = 0.0
            if cur_pos is not None:
                d_cur = self.dist3(cur_pos, goal)
                if d_cur < self.min_goal_distance:
                    continue
                dist_cost = float(self.goal_distance_penalty) * d_cur

            jump_cost = 0.0
            if old_goal is not None:
                d_jump = self.dist3(old_goal, goal)
                jump_cost = float(self.goal_jump_penalty) * d_jump

            value = raw_score / (1.0 + dist_cost + jump_cost)

            if value > best_value:
                best_value = value
                best = (goal, raw_score, value)

        if best is None:
            return

        new_goal, raw_score, value = best

        # 6. 目标保持：短时间内不随便切换。
        if self.current_goal is not None:
            hold_not_expired = (now - self.current_goal_time) < self.goal_hold_time
            switch_needed = value > self.current_goal_value * self.goal_switch_ratio

            if hold_not_expired and not switch_needed:
                goal_to_pub = self.current_goal
                raw_score_to_pub = self.current_goal_raw_score
                value_to_pub = self.current_goal_value
                switched = False
            else:
                smoothed = self.smooth_goal(self.current_goal, new_goal)
                limited = self.limit_goal_step(self.current_goal, smoothed)

                self.current_goal = limited
                self.current_goal_raw_score = raw_score
                self.current_goal_value = value
                self.current_goal_time = now

                goal_to_pub = limited
                raw_score_to_pub = raw_score
                value_to_pub = value
                switched = True
        else:
            self.current_goal = new_goal
            self.current_goal_raw_score = raw_score
            self.current_goal_value = value
            self.current_goal_time = now

            goal_to_pub = new_goal
            raw_score_to_pub = raw_score
            value_to_pub = value
            switched = True

        x, y, z = goal_to_pub

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = self.frame_id
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.w = 1.0

        self.pub_goal.publish(goal_msg)

        debug = {
            "event": "stable_next_uav_search_goal",
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2),
            "raw_score": round(raw_score_to_pub, 3),
            "goal_value": round(value_to_pub, 3),
            "max_score": round(max_score, 3),
            "cluster_size": int(len(cluster_idx)),
            "switched": bool(switched),
            "yaw_deg": round(math.degrees(self.current_yaw), 2)
        }

        self.pub_debug.publish(String(data=json.dumps(debug, ensure_ascii=False)))

        rospy.loginfo_throttle(
            1.0,
            "Stable next goal: x=%.2f y=%.2f z=%.2f raw=%.3f value=%.3f max=%.3f cluster=%d switched=%s",
            x,
            y,
            z,
            raw_score_to_pub,
            value_to_pub,
            max_score,
            len(cluster_idx),
            str(switched)
        )


if __name__ == "__main__":
    rospy.init_node("uav_3d_search_fusion_node", anonymous=False)
    node = UAV3DSearchFusionNode()
    rospy.spin()



