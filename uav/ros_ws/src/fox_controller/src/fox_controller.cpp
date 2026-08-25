#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cctype>

// 使用二进制掩码定义控制模式：速度控制 + yaw 控制
// 使用 VX, VY, VZ 和 YAW
#define VELOCITY2D_CONTROL 0b101111000111

static constexpr double FOX_PI = 3.14159265358979323846;

class FoxControllerLite {
public:
    FoxControllerLite();

    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

    void controlLoop(const ros::TimerEvent& event);
    bool takeoffLandCallback(std_srvs::SetBool::Request &req,
                             std_srvs::SetBool::Response &res);

private:
    struct Waypoint {
        double x{0.0};
        double y{0.0};
        double z{1.0};
        double yaw{0.0};       // 单位：rad。用于到达航点后悬停朝向
        bool yaw_valid{false}; // true 表示航点文件/参数中提供了 yaw
        double hover_time{1.0};       // 单位：s。该航点到达后的停留时间
        bool hover_time_valid{false}; // true 表示航点文件/参数中提供了停留时间
    };

    bool loadPresetWaypointsFromParams(bool verbose);
    bool loadWaypointsFromFile(const std::string& path, bool verbose);
    bool setTargetFromWaypointIndex(int index);
    bool publishWaypointGoal(int index, bool force_log = true);
    void startPresetWaypointMission();
    double getCurrentWaypointHoverTime() const;
    void holdAtGoalPoint();
    void switchToLanding(const std::string& reason);

    // ROS 句柄和定时器
    ros::NodeHandle nh_;
    ros::NodeHandle nh_global_;
    ros::Timer control_timer_;

    // 订阅器
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber position_cmd_sub_;
    ros::Subscriber rc_sub_;

    // 发布器
    ros::Publisher setpoint_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher waypoint_goal_pub_;     // 逐个航点发布给 EGO-Planner 手动目标模式
    ros::Publisher mission_cmd_pub_;       // 降落/急停时通知 mission_supervisor 停止继续发布航点
    tf::TransformBroadcaster tf_broadcaster_;

    // 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceServer takeoff_land_service_;

    // 状态变量
    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;
    bool has_odom_{false};
    bool has_target_{false};
    bool allow_yaw_{true};

    // =========================
    // launch 航点顺序执行参数
    // =========================
    bool use_preset_waypoints_{true};
    bool use_rviz_goal_{true};
    bool auto_track_preset_after_takeoff_{true};
    bool auto_land_after_mission_{true};
    int drone_id_{0};
    int planner_flight_type_{-1};
    std::string ego_planner_node_name_;     // 默认 /drone_0_ego_planner_node
    std::string waypoints_file_;             // 可选：从 txt/csv 文件读取航点
    std::string mission_cmd_topic_{"/dk_mission_cmd"}; // 通知 uav_mission_supervisor 停止任务
    std::vector<Waypoint> preset_waypoints_;
    bool preset_waypoints_loaded_{false};
    bool mission_active_{false};
    bool mission_finished_{false};
    int current_waypoint_index_{0};
    double waypoint_hover_time_{1.0};       // 默认悬停时间，单位 s；TXT/参数中每个航点可覆盖
    double waypoint_goal_republish_interval_{0.5};
    ros::Time waypoint_hover_start_time_;
    ros::Time last_waypoint_goal_pub_time_;

    // 目标航点记录。当前正在飞向的目标点。
    geometry_msgs::PoseStamped target_pose_;
    bool target_z_valid_{false};
    double target_yaw_{0.0};

    // 到达目标后的悬停点
    double goal_hover_x_{0.0};
    double goal_hover_y_{0.0};
    double goal_hover_z_{0.0};
    double goal_hover_yaw_{0.0};

    // 到达判定计时，要求连续满足阈值一段时间，避免瞬间误判。
    bool arrival_check_started_{false};
    ros::Time arrival_check_start_time_;

    // 到达目标航点判定与悬停参数
    double goal_reach_xy_threshold_{0.30};   // XY 平面到达阈值，单位 m
    double goal_reach_z_threshold_{0.25};    // Z 向到达阈值，单位 m
    double goal_reach_vel_threshold_{0.30};  // 到达时速度阈值，单位 m/s
    double goal_reach_hold_time_{1.0};       // 连续满足到达条件的时间，单位 s；不是悬停时间
    double goal_hover_kp_{1.0};              // 目标点悬停 P 控制增益
    double goal_hover_max_vel_{0.6};         // 悬停修正最大速度，单位 m/s

    // 初始位置记录
    struct InitPosition {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
        bool recorded{false};
    } init_pos_;

    // EGO-Planner 指令
    quadrotor_msgs::PositionCommand ego_command_;
    bool has_ego_command_{false};

    // 控制参数
    const unsigned short velocity_mask_{VELOCITY2D_CONTROL};
    mavros_msgs::PositionTarget current_setpoint_;

    // 飞行状态机
    enum FlightState {
        WAITING,
        TAKING_OFF,
        HOVERING,           // 起飞点悬停，等待任务
        TRACKING,           // 跟踪 EGO-Planner 指令，飞向当前航点
        WAYPOINT_HOVERING,  // 到达当前航点后悬停 waypoint_hover_time_ 秒
        MISSION_FINISHED,   // 所有航点完成后，如果不自动降落，则停在最后一点
        LANDING
    };

    FlightState flight_state_{WAITING};

    // 起飞参数
    double takeoff_height_{1.6};
    ros::Time last_mode_request_time_;

    // =========================
    // RC CH5 起飞/降落参数
    // =========================
    bool enable_rc_takeoff_{true};
    int rc_takeoff_channel_{4};              // CH5 -> channels[4]
    int rc_high_threshold_{1700};
    int rc_low_threshold_{1300};
    bool rc_initialized_{false};
    bool rc_last_high_{false};
    double rc_action_cooldown_{1.0};
    ros::Time last_rc_action_time_;
};

FoxControllerLite::FoxControllerLite() : nh_("~"), nh_global_() {
    // 基础参数
    nh_.param("allow_yaw", allow_yaw_, true);
    nh_.param("takeoff_height", takeoff_height_, 1.6);

    // launch 航点顺序执行参数
    nh_.param("use_preset_waypoints", use_preset_waypoints_, true);
    nh_.param("use_rviz_goal", use_rviz_goal_, true);
    nh_.param("auto_track_preset_after_takeoff", auto_track_preset_after_takeoff_, true);
    nh_.param("auto_land_after_mission", auto_land_after_mission_, true);
    nh_.param("waypoint_hover_time", waypoint_hover_time_, 1.0);
    nh_.param("waypoint_goal_republish_interval", waypoint_goal_republish_interval_, 0.5);
    nh_.param("drone_id", drone_id_, 0);
    nh_.param<std::string>("waypoints_file", waypoints_file_, std::string(""));
    nh_.param<std::string>("mission_cmd_topic", mission_cmd_topic_, std::string("/dk_mission_cmd"));

    ego_planner_node_name_ = "/drone_" + std::to_string(drone_id_) + "_ego_planner_node";
    nh_.param<std::string>("ego_planner_node_name", ego_planner_node_name_, ego_planner_node_name_);
    if (!ego_planner_node_name_.empty() && ego_planner_node_name_[0] != '/') {
        ego_planner_node_name_ = "/" + ego_planner_node_name_;
    }

    // 到达目标航点后的悬停参数
    nh_.param("goal_reach_xy_threshold", goal_reach_xy_threshold_, 0.30);
    nh_.param("goal_reach_z_threshold", goal_reach_z_threshold_, 0.25);
    nh_.param("goal_reach_vel_threshold", goal_reach_vel_threshold_, 0.30);
    nh_.param("goal_reach_hold_time", goal_reach_hold_time_, 1.0);
    nh_.param("goal_hover_kp", goal_hover_kp_, 1.0);
    nh_.param("goal_hover_max_vel", goal_hover_max_vel_, 0.6);

    // RC 起飞参数
    nh_.param("enable_rc_takeoff", enable_rc_takeoff_, true);
    nh_.param("rc_takeoff_channel", rc_takeoff_channel_, 4);
    nh_.param("rc_high_threshold", rc_high_threshold_, 1700);
    nh_.param("rc_low_threshold", rc_low_threshold_, 1300);
    nh_.param("rc_action_cooldown", rc_action_cooldown_, 1.0);

    // 订阅器
    state_sub_ = nh_.subscribe("/mavros/state", 10,
                               &FoxControllerLite::stateCallback, this);

    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10,
                              &FoxControllerLite::odometryCallback, this);

    // 保留 RViz 手动目标备用；自动航点发布到同一个话题，但会用 frame_id 标记并在回调中忽略自发消息。
    target_sub_ = nh_.subscribe("/move_base_simple/goal", 10,
                                &FoxControllerLite::targetCallback, this);

    position_cmd_sub_ = nh_.subscribe("/position_cmd", 10,
                                      &FoxControllerLite::positionCommandCallback, this);

    rc_sub_ = nh_.subscribe("/mavros/rc/in", 10,
                            &FoxControllerLite::rcCallback, this);

    // 发布器
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/track_drone_point", 5);

    // EGO-Planner flight_type=1 时订阅 /move_base_simple/goal。
    waypoint_goal_pub_ = nh_global_.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, false);

    // 通知 Jetson 端 uav_mission_supervisor 停止当前 DK2500 航线，防止 CH5 降落时仍继续执行返程航点。
    mission_cmd_pub_ = nh_global_.advertise<std_msgs::String>(
        mission_cmd_topic_, 5, false);

    // MAVROS 服务客户端
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    // 保留原来的起飞/降落服务，方便终端调试
    takeoff_land_service_ = nh_.advertiseService(
        "takeoff_land", &FoxControllerLite::takeoffLandCallback, this);

    // 控制定时器，50Hz
    control_timer_ = nh_.createTimer(
        ros::Duration(0.02), &FoxControllerLite::controlLoop, this);

    ROS_INFO("fox_controller_lite initialized.");
    ROS_INFO("Takeoff height: %.2f m", takeoff_height_);
    ROS_INFO("Mission command topic for STOP_TASK: %s", mission_cmd_topic_.c_str());
    ROS_INFO("Sequential waypoint mission: use_preset=%s, auto_after_takeoff=%s, auto_land=%s, hover_each_wp=%.2f s, ego_node=%s",
             use_preset_waypoints_ ? "true" : "false",
             auto_track_preset_after_takeoff_ ? "true" : "false",
             auto_land_after_mission_ ? "true" : "false",
             waypoint_hover_time_,
             ego_planner_node_name_.c_str());
    if (!waypoints_file_.empty()) {
        ROS_INFO("Waypoint file: %s", waypoints_file_.c_str());
    }
    ROS_INFO("Reach check: xy_thr=%.2f m, z_thr=%.2f m, vel_thr=%.2f m/s, hold=%.2f s; hover kp=%.2f, max_vel=%.2f m/s",
             goal_reach_xy_threshold_, goal_reach_z_threshold_, goal_reach_vel_threshold_,
             goal_reach_hold_time_, goal_hover_kp_, goal_hover_max_vel_);

    if (use_preset_waypoints_) {
        loadPresetWaypointsFromParams(true);
    }
}

bool FoxControllerLite::loadWaypointsFromFile(const std::string& path, bool verbose) {
    std::ifstream file(path.c_str());
    if (!file.is_open()) {
        if (verbose) {
            ROS_WARN("Failed to open waypoint file: %s", path.c_str());
        }
        return false;
    }

    std::vector<Waypoint> loaded;
    std::string line;
    int line_no = 0;

    while (std::getline(file, line)) {
        line_no++;

        // 去掉行首空白
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
            continue;
        }

        // 支持注释行：# 或 // 开头
        std::string trimmed = line.substr(first);
        if (trimmed[0] == '#') {
            continue;
        }
        if (trimmed.size() >= 2 && trimmed[0] == '/' && trimmed[1] == '/') {
            continue;
        }

        // 支持行尾注释
        size_t hash_pos = trimmed.find('#');
        if (hash_pos != std::string::npos) {
            trimmed = trimmed.substr(0, hash_pos);
        }
        size_t slash_pos = trimmed.find("//");
        if (slash_pos != std::string::npos) {
            trimmed = trimmed.substr(0, slash_pos);
        }

        // 支持逗号或空格分隔：x,y,z,yaw_deg,hover_time 或 x y z yaw_deg hover_time
        // yaw_deg 使用角度制，0 表示朝 +X，90 表示朝 +Y。
        // hover_time 使用秒，表示到达该航点后停留多久。
        for (char& c : trimmed) {
            if (c == ',') {
                c = ' ';
            }
        }

        std::stringstream ss(trimmed);
        Waypoint wp;
        wp.hover_time = waypoint_hover_time_;

        if (!(ss >> wp.x >> wp.y >> wp.z)) {
            ROS_WARN("Invalid waypoint line %d in %s. Expected: x y z [yaw_deg] [hover_time]. Content: %s",
                     line_no, path.c_str(), line.c_str());
            continue;
        }

        double yaw_deg = 0.0;
        if (ss >> yaw_deg) {
            wp.yaw = yaw_deg * FOX_PI / 180.0;
            wp.yaw_valid = true;
        } else {
            // 兼容旧格式：没有 yaw 时，到达后沿用起飞时的初始朝向。
            wp.yaw = 0.0;
            wp.yaw_valid = false;
        }

        double hover_time = waypoint_hover_time_;
        if (ss >> hover_time) {
            if (hover_time < 0.0) {
                ROS_WARN("Waypoint line %d has negative hover_time %.2f. Clamped to 0.0 s.",
                         line_no, hover_time);
                hover_time = 0.0;
            }
            wp.hover_time = hover_time;
            wp.hover_time_valid = true;
        }

        loaded.push_back(wp);
    }

    if (loaded.empty()) {
        ROS_WARN("Waypoint file is empty or has no valid waypoint: %s", path.c_str());
        return false;
    }

    if (loaded.size() > 50) {
        ROS_WARN("Waypoint file has %lu waypoints. Only the first 50 will be used.", loaded.size());
        loaded.resize(50);
    }

    preset_waypoints_ = loaded;
    preset_waypoints_loaded_ = true;

    ROS_INFO("Loaded %lu waypoint(s) from file: %s", preset_waypoints_.size(), path.c_str());
    for (size_t i = 0; i < preset_waypoints_.size(); ++i) {
        const Waypoint& wp = preset_waypoints_[i];
        const double hover_s = wp.hover_time_valid ? wp.hover_time : waypoint_hover_time_;
        if (wp.yaw_valid) {
            ROS_INFO("  waypoint%lu = [%.2f, %.2f, %.2f], yaw=%.1f deg, hover=%.2f s",
                     i, wp.x, wp.y, wp.z, wp.yaw * 180.0 / FOX_PI, hover_s);
        } else {
            ROS_INFO("  waypoint%lu = [%.2f, %.2f, %.2f], yaw=init, hover=%.2f s",
                     i, wp.x, wp.y, wp.z, hover_s);
        }
    }

    return true;
}

bool FoxControllerLite::loadPresetWaypointsFromParams(bool verbose) {
    preset_waypoints_.clear();
    preset_waypoints_loaded_ = false;
    planner_flight_type_ = -1;

    // 优先从外部 txt/csv 文件读取航点；文件为空或读取失败时，再回退到 launch/ROS 参数。
    if (!waypoints_file_.empty()) {
        if (loadWaypointsFromFile(waypoints_file_, verbose)) {
            return true;
        }
        ROS_WARN("Falling back to ROS waypoint params because waypoint file could not be loaded.");
    }

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_global;

    // 优先从 fox_controller 自己的私有参数读取；没有再读 EGO-Planner 节点私有参数。
    std::vector<std::string> bases;
    bases.push_back("~");
    bases.push_back(ego_planner_node_name_);

    int waypoint_num = -1;
    std::string selected_base;

    for (const auto& base : bases) {
        int n = -1;
        bool ok = false;

        if (base == "~") {
            ok = nh_private.getParam("fsm/waypoint_num", n) || nh_private.getParam("waypoint_num", n);
            if (ok) {
                int ft = -1;
                if (nh_private.getParam("fsm/flight_type", ft) || nh_private.getParam("flight_type", ft)) {
                    planner_flight_type_ = ft;
                }
            }
        } else {
            ok = nh_global.getParam(base + "/fsm/waypoint_num", n);
            if (ok) {
                int ft = -1;
                if (nh_global.getParam(base + "/fsm/flight_type", ft)) {
                    planner_flight_type_ = ft;
                }
            }
        }

        if (ok && n > 0) {
            waypoint_num = n;
            selected_base = base;
            break;
        }
    }

    if (waypoint_num <= 0) {
        if (verbose) {
            ROS_WARN("No preset waypoints found. Tried private params and %s/fsm/waypoint_num.",
                     ego_planner_node_name_.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0, "Waiting for preset waypoint params from %s ...",
                              ego_planner_node_name_.c_str());
        }
        return false;
    }

    if (waypoint_num > 50) {
        ROS_WARN("waypoint_num=%d is too large. Only the first 50 waypoints will be used.", waypoint_num);
        waypoint_num = 50;
    }

    for (int i = 0; i < waypoint_num; ++i) {
        Waypoint wp;
        bool ok = false;

        if (selected_base == "~") {
            ok =
                (nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_x", wp.x) ||
                 nh_private.getParam("waypoint" + std::to_string(i) + "_x", wp.x)) &&
                (nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_y", wp.y) ||
                 nh_private.getParam("waypoint" + std::to_string(i) + "_y", wp.y)) &&
                (nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_z", wp.z) ||
                 nh_private.getParam("waypoint" + std::to_string(i) + "_z", wp.z));
        } else {
            ok =
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_x", wp.x) &&
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_y", wp.y) &&
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_z", wp.z);
        }

        if (!ok) {
            ROS_WARN("Failed to read waypoint%d from %s. Preset waypoint mode disabled.",
                     i, selected_base.c_str());
            preset_waypoints_.clear();
            return false;
        }

        // 可选 yaw 参数，单位：度。支持 waypoint0_yaw / waypoint0_yaw_deg。
        double yaw_deg = 0.0;
        bool yaw_ok = false;
        if (selected_base == "~") {
            yaw_ok =
                nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_yaw", yaw_deg) ||
                nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_yaw_deg", yaw_deg) ||
                nh_private.getParam("waypoint" + std::to_string(i) + "_yaw", yaw_deg) ||
                nh_private.getParam("waypoint" + std::to_string(i) + "_yaw_deg", yaw_deg);
        } else {
            yaw_ok =
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_yaw", yaw_deg) ||
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_yaw_deg", yaw_deg);
        }

        if (yaw_ok) {
            wp.yaw = yaw_deg * FOX_PI / 180.0;
            wp.yaw_valid = true;
        }

        // 可选停留时间参数，单位：秒。支持 waypoint0_hover_time / waypoint0_hover_s。
        double hover_time = waypoint_hover_time_;
        bool hover_ok = false;
        if (selected_base == "~") {
            hover_ok =
                nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_hover_time", hover_time) ||
                nh_private.getParam("fsm/waypoint" + std::to_string(i) + "_hover_s", hover_time) ||
                nh_private.getParam("waypoint" + std::to_string(i) + "_hover_time", hover_time) ||
                nh_private.getParam("waypoint" + std::to_string(i) + "_hover_s", hover_time);
        } else {
            hover_ok =
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_hover_time", hover_time) ||
                nh_global.getParam(selected_base + "/fsm/waypoint" + std::to_string(i) + "_hover_s", hover_time);
        }

        if (hover_ok) {
            if (hover_time < 0.0) {
                ROS_WARN("waypoint%d hover_time %.2f is negative. Clamped to 0.0 s.", i, hover_time);
                hover_time = 0.0;
            }
            wp.hover_time = hover_time;
            wp.hover_time_valid = true;
        } else {
            wp.hover_time = waypoint_hover_time_;
            wp.hover_time_valid = false;
        }

        preset_waypoints_.push_back(wp);
    }

    preset_waypoints_loaded_ = !preset_waypoints_.empty();

    if (preset_waypoints_loaded_) {
        ROS_INFO("Loaded %lu preset waypoint(s) from %s. flight_type=%d",
                 preset_waypoints_.size(), selected_base.c_str(), planner_flight_type_);
        for (size_t i = 0; i < preset_waypoints_.size(); ++i) {
            const Waypoint& wp = preset_waypoints_[i];
            const double hover_s = wp.hover_time_valid ? wp.hover_time : waypoint_hover_time_;
            if (wp.yaw_valid) {
                ROS_INFO("  waypoint%lu = [%.2f, %.2f, %.2f], yaw=%.1f deg, hover=%.2f s",
                         i, wp.x, wp.y, wp.z, wp.yaw * 180.0 / FOX_PI, hover_s);
            } else {
                ROS_INFO("  waypoint%lu = [%.2f, %.2f, %.2f], yaw=init, hover=%.2f s",
                         i, wp.x, wp.y, wp.z, hover_s);
            }
        }

        if (planner_flight_type_ == 2) {
            ROS_WARN("EGO-Planner flight_type=2 will auto-chain waypoints internally, so fox_controller cannot force a 1s stop at each waypoint. Use flight_type=1 for sequential stop-and-go mission.");
        } else if (planner_flight_type_ != 1) {
            ROS_WARN("EGO-Planner flight_type is %d. Sequential waypoint stop-and-go expects flight_type=1.", planner_flight_type_);
        }
    }

    return preset_waypoints_loaded_;
}

bool FoxControllerLite::setTargetFromWaypointIndex(int index) {
    if (!preset_waypoints_loaded_ || index < 0 || index >= static_cast<int>(preset_waypoints_.size())) {
        return false;
    }

    const Waypoint& wp = preset_waypoints_[index];

    target_pose_.header.stamp = ros::Time::now();
    target_pose_.header.frame_id = "world";
    target_pose_.pose.position.x = wp.x;
    target_pose_.pose.position.y = wp.y;
    target_pose_.pose.position.z = wp.z;

    const double wp_yaw = wp.yaw_valid ? wp.yaw : init_pos_.yaw;
    target_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(wp_yaw);

    target_z_valid_ = true;
    target_yaw_ = wp_yaw;
    has_target_ = true;
    arrival_check_started_ = false;

    const double hover_s = wp.hover_time_valid ? wp.hover_time : waypoint_hover_time_;
    ROS_WARN("Current mission target: waypoint%d [%.2f, %.2f, %.2f], hover_yaw=%.1f deg, hover_time=%.2f s",
             index, wp.x, wp.y, wp.z, target_yaw_ * 180.0 / FOX_PI, hover_s);
    return true;
}

bool FoxControllerLite::publishWaypointGoal(int index, bool force_log) {
    if (!preset_waypoints_loaded_ || index < 0 || index >= static_cast<int>(preset_waypoints_.size())) {
        return false;
    }

    const Waypoint& wp = preset_waypoints_[index];

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    // 用特殊 frame_id 标记为 fox_controller 自动发布，避免 targetCallback 把自己的消息当作 RViz 新目标。
    goal.header.frame_id = "fox_auto_waypoint";
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.position.z = wp.z;

    const double wp_yaw = wp.yaw_valid ? wp.yaw : init_pos_.yaw;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(wp_yaw);

    waypoint_goal_pub_.publish(goal);
    last_waypoint_goal_pub_time_ = ros::Time::now();

    if (force_log) {
        const double hover_s = wp.hover_time_valid ? wp.hover_time : waypoint_hover_time_;
        ROS_WARN("Published waypoint%d to /move_base_simple/goal: [%.2f, %.2f, %.2f], yaw=%.1f deg, hover=%.2f s",
                 index, wp.x, wp.y, wp.z, wp_yaw * 180.0 / FOX_PI, hover_s);
    }
    return true;
}

void FoxControllerLite::startPresetWaypointMission() {
    if (!preset_waypoints_loaded_) {
        if (!loadPresetWaypointsFromParams(false)) {
            return;
        }
    }

    if (preset_waypoints_.empty()) {
        ROS_WARN("Cannot start mission: no preset waypoints.");
        return;
    }

    current_waypoint_index_ = 0;
    mission_active_ = true;
    mission_finished_ = false;
    has_ego_command_ = false;
    arrival_check_started_ = false;

    setTargetFromWaypointIndex(current_waypoint_index_);
    publishWaypointGoal(current_waypoint_index_, true);

    flight_state_ = TRACKING;
    ROS_WARN("Sequential waypoint mission started. Total waypoints: %lu", preset_waypoints_.size());
}

double FoxControllerLite::getCurrentWaypointHoverTime() const {
    if (mission_active_ &&
        current_waypoint_index_ >= 0 &&
        current_waypoint_index_ < static_cast<int>(preset_waypoints_.size())) {
        const Waypoint& wp = preset_waypoints_[current_waypoint_index_];
        return wp.hover_time_valid ? wp.hover_time : waypoint_hover_time_;
    }
    return waypoint_hover_time_;
}

void FoxControllerLite::holdAtGoalPoint() {
    const auto& odom = current_odom_.pose.pose.position;

    double vx = goal_hover_kp_ * (goal_hover_x_ - odom.x);
    double vy = goal_hover_kp_ * (goal_hover_y_ - odom.y);
    double vz = goal_hover_kp_ * (goal_hover_z_ - odom.z);

    vx = std::max(-goal_hover_max_vel_, std::min(goal_hover_max_vel_, vx));
    vy = std::max(-goal_hover_max_vel_, std::min(goal_hover_max_vel_, vy));
    vz = std::max(-goal_hover_max_vel_, std::min(goal_hover_max_vel_, vz));

    current_setpoint_.velocity.x = vx;
    current_setpoint_.velocity.y = vy;
    current_setpoint_.velocity.z = vz;
    current_setpoint_.yaw = goal_hover_yaw_;
    current_setpoint_.type_mask = velocity_mask_;

    setpoint_pub_.publish(current_setpoint_);
}

void FoxControllerLite::switchToLanding(const std::string& reason) {
    // 先通知 mission_supervisor 停止当前 DK2500 航线。
    // 目的：CH5 回低降落时，不再继续发布 exhibition_return_xx / DK route goal，
    // 避免飞机在 AUTO.LAND 接管前仍被返程航点拉动。
    std_msgs::String stop_msg;
    stop_msg.data = "{\"cmd\":\"STOP_TASK\",\"reason\":\"fox_controller_landing\"}";
    mission_cmd_pub_.publish(stop_msg);

    flight_state_ = LANDING;
    has_target_ = false;
    mission_active_ = false;
    mission_finished_ = false;
    arrival_check_started_ = false;
    has_ego_command_ = false;

    // 让 LANDING 状态下一次 controlLoop 立即请求 AUTO.LAND，
    // 不再受前一次 OFFBOARD/ARM 请求的 5 秒冷却影响。
    last_mode_request_time_ = ros::Time(0);

    ROS_WARN("Switching to LANDING. Reason: %s. STOP_TASK sent to %s",
             reason.c_str(), mission_cmd_topic_.c_str());
}

// 起飞/降落服务回调
bool FoxControllerLite::takeoffLandCallback(std_srvs::SetBool::Request &req,
                                            std_srvs::SetBool::Response &res) {
    if (req.data) {
        // 起飞请求
        if (flight_state_ == WAITING) {
            if (!has_odom_) {
                res.success = false;
                res.message = "No odometry data";
                ROS_WARN("Takeoff service ignored: no odometry data.");
                return true;
            }

            flight_state_ = TAKING_OFF;
            ROS_INFO("Takeoff command received. Switching to TAKING_OFF state.");
            res.success = true;
            res.message = "Takeoff initiated";
            return true;
        } else {
            res.success = false;
            res.message = "Already in flight or taking off";
            return true;
        }
    } else {
        // 降落请求
        if (flight_state_ == TAKING_OFF ||
            flight_state_ == HOVERING ||
            flight_state_ == TRACKING ||
            flight_state_ == WAYPOINT_HOVERING ||
            flight_state_ == MISSION_FINISHED) {

            switchToLanding("takeoff_land service requested land");
            res.success = true;
            res.message = "Landing initiated";
            return true;
        } else {
            res.success = false;
            res.message = "Not in flight state";
            return true;
        }
    }
}

// MAVROS 状态回调
void FoxControllerLite::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

// RC 输入回调：CH5 低->高起飞，高->低降落
void FoxControllerLite::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
    if (!enable_rc_takeoff_) {
        return;
    }

    if (rc_takeoff_channel_ < 0) {
        ROS_WARN_THROTTLE(2.0, "Invalid rc_takeoff_channel: %d", rc_takeoff_channel_);
        return;
    }

    if (msg->channels.size() <= static_cast<size_t>(rc_takeoff_channel_)) {
        ROS_WARN_THROTTLE(2.0,
                          "RC channel index %d not available. channels size = %lu",
                          rc_takeoff_channel_, msg->channels.size());
        return;
    }

    const int pwm = msg->channels[rc_takeoff_channel_];

    // 带滞回判断，避免阈值附近抖动
    bool current_high = rc_last_high_;
    if (pwm > rc_high_threshold_) {
        current_high = true;
    } else if (pwm < rc_low_threshold_) {
        current_high = false;
    }

    // 首次接收 RC，只记录状态，不触发动作，防止节点启动时 CH5 已经在高位导致直接起飞
    if (!rc_initialized_) {
        rc_last_high_ = current_high;
        rc_initialized_ = true;
        last_rc_action_time_ = ros::Time::now();

        ROS_INFO("RC initialized. channel index=%d pwm=%d state=%s",
                 rc_takeoff_channel_, pwm, current_high ? "HIGH" : "LOW");

        if (current_high) {
            ROS_WARN("CH5 is already HIGH at startup. Move CH5 LOW first, then HIGH to trigger takeoff.");
        }
        return;
    }

    if (current_high == rc_last_high_) {
        return;
    }

    if (ros::Time::now() - last_rc_action_time_ < ros::Duration(rc_action_cooldown_)) {
        return;
    }

    last_rc_action_time_ = ros::Time::now();

    // 低位 -> 高位：触发起飞
    if (!rc_last_high_ && current_high) {
        if (flight_state_ == WAITING) {
            if (!has_odom_) {
                ROS_WARN("RC CH5 HIGH ignored: no odometry data.");
            } else if (!current_state_.connected) {
                ROS_WARN("RC CH5 HIGH ignored: MAVROS not connected.");
            } else {
                flight_state_ = TAKING_OFF;
                ROS_WARN("RC CH5 LOW->HIGH: TAKEOFF triggered. Switching to TAKING_OFF.");
            }
        } else {
            ROS_WARN("RC CH5 HIGH ignored: flight_state is not WAITING.");
        }
    }

    // 高位 -> 低位：触发降落
    if (rc_last_high_ && !current_high) {
        if (flight_state_ == TAKING_OFF ||
            flight_state_ == HOVERING ||
            flight_state_ == TRACKING ||
            flight_state_ == WAYPOINT_HOVERING ||
            flight_state_ == MISSION_FINISHED) {
            switchToLanding("RC CH5 HIGH->LOW");
        } else {
            ROS_WARN("RC CH5 LOW ignored: not in a flying state.");
        }
    }

    rc_last_high_ = current_high;
}

// 里程计回调
void FoxControllerLite::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
    has_odom_ = true;

    // 记录初始位置，仅第一次
    if (!init_pos_.recorded) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        init_pos_.yaw = yaw;
        init_pos_.x = msg->pose.pose.position.x;
        init_pos_.y = msg->pose.pose.position.y;
        init_pos_.z = msg->pose.pose.position.z;
        init_pos_.recorded = true;

        ROS_INFO("Initial position recorded: [%.2f, %.2f, %.2f] Yaw: %.2f",
                 init_pos_.x, init_pos_.y, init_pos_.z, init_pos_.yaw);
    }

    // 发布 drone_frame TF
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    ));

    transform.setRotation(tf::Quaternion(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    ));

    tf_broadcaster_.sendTransform(tf::StampedTransform(
        transform,
        ros::Time::now(),
        "world",
        "drone_frame"
    ));
}

// RViz 目标点回调。自动任务发布的目标会被忽略，避免自发自收破坏 mission_active_。
void FoxControllerLite::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (msg->header.frame_id == "fox_auto_waypoint") {
        return;
    }

    if (!use_rviz_goal_) {
        return;
    }

    if (flight_state_ == HOVERING ||
        flight_state_ == TRACKING ||
        flight_state_ == WAYPOINT_HOVERING ||
        flight_state_ == MISSION_FINISHED) {

        target_pose_ = *msg;
        has_target_ = true;
        arrival_check_started_ = false;
        has_ego_command_ = false;
        mission_active_ = false;
        mission_finished_ = false;

        // RViz 2D Nav Goal 通常 z=0。此时不把 z=0 作为无人机目标高度，
        // 只用 XY 判断是否到达；真正悬停高度在到达瞬间用当前高度锁定。
        target_z_valid_ = std::fabs(target_pose_.pose.position.z) > 0.05;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        target_yaw_ = yaw;

        if (flight_state_ == WAYPOINT_HOVERING || flight_state_ == MISSION_FINISHED) {
            flight_state_ = TRACKING;
        }

        ROS_INFO("Received RViz/manual target: [%.2f, %.2f, %.2f], z_valid=%s. Preset mission cancelled.",
                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                 target_z_valid_ ? "true" : "false");
    }
}

// EGO-Planner 轨迹指令回调
void FoxControllerLite::positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (flight_state_ == TRACKING) {
        ego_command_ = *msg;
        has_ego_command_ = true;
    }
}

// 主控制循环
void FoxControllerLite::controlLoop(const ros::TimerEvent& event) {
    if (!has_odom_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for odometry data...");
        return;
    }

    // 准备控制指令头
    current_setpoint_.header.stamp = ros::Time::now();
    current_setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_setpoint_.type_mask = velocity_mask_;

    const auto& odom = current_odom_.pose.pose.position;

    switch (flight_state_) {
        case WAITING: {
            // 等待状态：不发送控制指令
            ROS_INFO_THROTTLE(1.0, "Waiting for CH5 takeoff command...");
            break;
        }

        case TAKING_OFF: {
            // 起飞过程：切 OFFBOARD、解锁、飞到指定高度
            if (current_state_.mode != "OFFBOARD") {
                if (ros::Time::now() - last_mode_request_time_ > ros::Duration(5.0)) {
                    mavros_msgs::SetMode offboard_set_mode;
                    offboard_set_mode.request.custom_mode = "OFFBOARD";

                    if (set_mode_client_.call(offboard_set_mode)) {
                        if (offboard_set_mode.response.mode_sent) {
                            ROS_INFO("Offboard enabled");
                        } else {
                            ROS_WARN("Failed to send OFFBOARD mode.");
                        }
                    } else {
                        ROS_WARN("Failed to call /mavros/set_mode.");
                    }
                    last_mode_request_time_ = ros::Time::now();
                }

                current_setpoint_.position.x = init_pos_.x;
                current_setpoint_.position.y = init_pos_.y;
                current_setpoint_.position.z = init_pos_.z + takeoff_height_;
                current_setpoint_.yaw = init_pos_.yaw;
                current_setpoint_.type_mask = 0b100111111000; // 位置控制模式
                setpoint_pub_.publish(current_setpoint_);
            }
            else if (!current_state_.armed) {
                if (ros::Time::now() - last_mode_request_time_ > ros::Duration(5.0)) {
                    mavros_msgs::CommandBool arm_cmd;
                    arm_cmd.request.value = true;

                    if (arming_client_.call(arm_cmd)) {
                        if (arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                        } else {
                            ROS_WARN("Arming command rejected.");
                        }
                    } else {
                        ROS_WARN("Failed to call /mavros/cmd/arming.");
                    }
                    last_mode_request_time_ = ros::Time::now();
                }

                current_setpoint_.position.x = init_pos_.x;
                current_setpoint_.position.y = init_pos_.y;
                current_setpoint_.position.z = init_pos_.z + takeoff_height_;
                current_setpoint_.yaw = init_pos_.yaw;
                current_setpoint_.type_mask = 0b100111111000; // 位置控制模式
                setpoint_pub_.publish(current_setpoint_);
            }
            else if (odom.z >= init_pos_.z + takeoff_height_ - 0.1) {
                flight_state_ = HOVERING;
                ros::param::set("takeoff_flag", true);
                ROS_INFO("Takeoff completed. Switching to HOVERING state.");
            }
            else {
                current_setpoint_.position.x = init_pos_.x;
                current_setpoint_.position.y = init_pos_.y;
                current_setpoint_.position.z = init_pos_.z + takeoff_height_;
                current_setpoint_.yaw = init_pos_.yaw;
                current_setpoint_.type_mask = 0b100111111000; // 位置控制模式
                setpoint_pub_.publish(current_setpoint_);
            }
            break;
        }

        case HOVERING: {
            // 起飞点悬停；若启用 launch 航点任务，则自动发布第一个航点。
            const double kP = 1.0;
            current_setpoint_.velocity.x = kP * (init_pos_.x - odom.x);
            current_setpoint_.velocity.y = kP * (init_pos_.y - odom.y);
            current_setpoint_.velocity.z = kP * (init_pos_.z + takeoff_height_ - odom.z);
            current_setpoint_.yaw = init_pos_.yaw;
            current_setpoint_.type_mask = velocity_mask_;
            setpoint_pub_.publish(current_setpoint_);

            if (use_preset_waypoints_ && !preset_waypoints_loaded_) {
                loadPresetWaypointsFromParams(false);
            }

            if (use_preset_waypoints_ && preset_waypoints_loaded_ && auto_track_preset_after_takeoff_) {
                startPresetWaypointMission();
            } else if (has_target_) {
                has_ego_command_ = false;
                flight_state_ = TRACKING;
                ROS_INFO("Manual target received. Switching to TRACKING state.");
            } else {
                ROS_INFO_THROTTLE(2.0, "Hovering at takeoff height. Waiting for preset waypoints or RViz target...");
            }
            break;
        }

        case TRACKING: {
            // 跟踪 EGO-Planner 的 /position_cmd
            const double vel_gain = 0.8;
            const double pos_gain = 1.3;

            // 刚发布新航点时，EGO-Planner 需要一点时间生成 /position_cmd。
            // 第一条指令到来前保持起飞点或上一个悬停点，避免 ego_command_ 默认值把飞机拉向原点。
            if (!has_ego_command_) {
                if (mission_active_) {
                    const double now_sec = ros::Time::now().toSec();
                    const double last_sec = last_waypoint_goal_pub_time_.isZero() ? 0.0 : last_waypoint_goal_pub_time_.toSec();
                    if (now_sec - last_sec > waypoint_goal_republish_interval_) {
                        publishWaypointGoal(current_waypoint_index_, false);
                    }
                }

                const double hold_x = mission_active_ && current_waypoint_index_ > 0 ? goal_hover_x_ : init_pos_.x;
                const double hold_y = mission_active_ && current_waypoint_index_ > 0 ? goal_hover_y_ : init_pos_.y;
                const double hold_z = mission_active_ && current_waypoint_index_ > 0 ? goal_hover_z_ : init_pos_.z + takeoff_height_;

                const double kP_wait = 1.0;
                current_setpoint_.velocity.x = kP_wait * (hold_x - odom.x);
                current_setpoint_.velocity.y = kP_wait * (hold_y - odom.y);
                current_setpoint_.velocity.z = kP_wait * (hold_z - odom.z);
                current_setpoint_.yaw = init_pos_.yaw;
                current_setpoint_.type_mask = velocity_mask_;
                setpoint_pub_.publish(current_setpoint_);

                ROS_WARN_THROTTLE(1.0, "TRACKING: waiting for first /position_cmd from EGO-Planner...");
                break;
            }

            current_setpoint_.velocity.x =
                vel_gain * ego_command_.velocity.x +
                pos_gain * (ego_command_.position.x - odom.x);

            current_setpoint_.velocity.y =
                vel_gain * ego_command_.velocity.y +
                pos_gain * (ego_command_.position.y - odom.y);

            current_setpoint_.velocity.z =
                pos_gain * (ego_command_.position.z - odom.z);

            current_setpoint_.yaw = allow_yaw_ ? ego_command_.yaw : init_pos_.yaw;
            current_setpoint_.type_mask = velocity_mask_;
            setpoint_pub_.publish(current_setpoint_);

            // 发布可视化 marker：当前 EGO 指令点
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = ego_command_.position.x;
            marker.pose.position.y = ego_command_.position.y;
            marker.pose.position.z = ego_command_.position.z;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_pub_.publish(marker);

            // =========================
            // 到达当前航点判定
            // =========================
            if (has_target_) {
                const double dx_goal = target_pose_.pose.position.x - odom.x;
                const double dy_goal = target_pose_.pose.position.y - odom.y;
                const double dz_goal = target_pose_.pose.position.z - odom.z;

                const double xy_error = std::hypot(dx_goal, dy_goal);
                const double z_error = std::fabs(dz_goal);

                const double vx = current_odom_.twist.twist.linear.x;
                const double vy = current_odom_.twist.twist.linear.y;
                const double vz = current_odom_.twist.twist.linear.z;
                const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

                const bool xy_reached = xy_error < goal_reach_xy_threshold_;
                const bool z_reached = (!target_z_valid_) || (z_error < goal_reach_z_threshold_);
                const bool speed_low = speed < goal_reach_vel_threshold_;

                if (xy_reached && z_reached && speed_low) {
                    if (!arrival_check_started_) {
                        arrival_check_started_ = true;
                        arrival_check_start_time_ = ros::Time::now();
                    } else if (ros::Time::now() - arrival_check_start_time_ >
                               ros::Duration(goal_reach_hold_time_)) {

                        goal_hover_x_ = target_pose_.pose.position.x;
                        goal_hover_y_ = target_pose_.pose.position.y;
                        goal_hover_z_ = target_z_valid_ ? target_pose_.pose.position.z : odom.z;
                        goal_hover_yaw_ = allow_yaw_ ? target_yaw_ : init_pos_.yaw;

                        has_target_ = false;
                        has_ego_command_ = false;
                        arrival_check_started_ = false;
                        waypoint_hover_start_time_ = ros::Time::now();
                        flight_state_ = WAYPOINT_HOVERING;

                        if (mission_active_) {
                            const double hover_s = getCurrentWaypointHoverTime();
                            ROS_WARN("Reached waypoint%d. Hovering %.2f s at [%.2f, %.2f, %.2f], yaw=%.1f deg.",
                                     current_waypoint_index_, hover_s,
                                     goal_hover_x_, goal_hover_y_, goal_hover_z_,
                                     goal_hover_yaw_ * 180.0 / FOX_PI);
                        } else {
                            ROS_WARN("Reached manual target. Hovering at [%.2f, %.2f, %.2f].",
                                     goal_hover_x_, goal_hover_y_, goal_hover_z_);
                        }
                    }
                } else {
                    arrival_check_started_ = false;
                }

                ROS_INFO_THROTTLE(
                    1.0,
                    "Tracking | wp=%d/%lu | cmd_speed=%.2f m/s | goal_xy_err=%.2f m | goal_z_err=%.2f m | odom_speed=%.2f m/s",
                    mission_active_ ? current_waypoint_index_ : -1,
                    preset_waypoints_.size(),
                    std::hypot(current_setpoint_.velocity.x, current_setpoint_.velocity.y),
                    xy_error,
                    z_error,
                    speed
                );
            }
            break;
        }

        case WAYPOINT_HOVERING: {
            // 到达当前航点后，真正悬停 waypoint_hover_time_ 秒。
            holdAtGoalPoint();

            const double elapsed = (ros::Time::now() - waypoint_hover_start_time_).toSec();

            if (mission_active_) {
                const double hover_s = getCurrentWaypointHoverTime();
                ROS_INFO_THROTTLE(
                    1.0,
                    "WAYPOINT_HOVERING | waypoint%d/%lu | %.2f / %.2f s | hold=[%.2f, %.2f, %.2f], yaw=%.1f deg",
                    current_waypoint_index_, preset_waypoints_.size(), elapsed, hover_s,
                    goal_hover_x_, goal_hover_y_, goal_hover_z_, goal_hover_yaw_ * 180.0 / FOX_PI
                );

                if (elapsed >= hover_s) {
                    if (current_waypoint_index_ + 1 < static_cast<int>(preset_waypoints_.size())) {
                        current_waypoint_index_++;
                        setTargetFromWaypointIndex(current_waypoint_index_);
                        publishWaypointGoal(current_waypoint_index_, true);
                        has_ego_command_ = false;
                        arrival_check_started_ = false;
                        flight_state_ = TRACKING;
                        ROS_WARN("Moving to next waypoint%d.", current_waypoint_index_);
                    } else {
                        mission_active_ = false;
                        mission_finished_ = true;
                        if (auto_land_after_mission_) {
                            switchToLanding("all preset waypoints completed");
                        } else {
                            flight_state_ = MISSION_FINISHED;
                            ROS_WARN("All preset waypoints completed. Auto land disabled; holding final waypoint.");
                        }
                    }
                }
            } else {
                // RViz 手动目标到达后，不自动降落，保持悬停。
                ROS_INFO_THROTTLE(
                    2.0,
                    "MANUAL_TARGET_HOVERING | hold=[%.2f, %.2f, %.2f] | elapsed=%.2f s",
                    goal_hover_x_, goal_hover_y_, goal_hover_z_, elapsed
                );
            }
            break;
        }

        case MISSION_FINISHED: {
            // 全部航点完成，若不自动降落则一直保持最后一点。
            holdAtGoalPoint();
            ROS_INFO_THROTTLE(
                2.0,
                "MISSION_FINISHED | holding final waypoint [%.2f, %.2f, %.2f], yaw=%.1f deg",
                goal_hover_x_, goal_hover_y_, goal_hover_z_, goal_hover_yaw_ * 180.0 / FOX_PI
            );
            break;
        }

        case LANDING: {
            // 降落过程：切换 AUTO.LAND
            if (current_state_.mode != "AUTO.LAND") {
                if (ros::Time::now() - last_mode_request_time_ > ros::Duration(5.0)) {
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";

                    if (set_mode_client_.call(land_set_mode)) {
                        if (land_set_mode.response.mode_sent) {
                            ROS_INFO("Landing mode enabled");
                        } else {
                            ROS_WARN("Failed to send AUTO.LAND mode.");
                        }
                    } else {
                        ROS_WARN("Failed to call /mavros/set_mode for AUTO.LAND.");
                    }
                    last_mode_request_time_ = ros::Time::now();
                }
            }

            // 检查是否接近地面
            if (current_state_.armed && odom.z < init_pos_.z + 0.1) {
                ros::Duration(2.0).sleep();
                flight_state_ = WAITING;
                has_target_ = false;
                mission_active_ = false;
                mission_finished_ = false;
                arrival_check_started_ = false;
                has_ego_command_ = false;
                ROS_INFO("Landing completed. Switching to WAITING state.");
            }
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fox_controller");
    FoxControllerLite controller;
    ros::spin();
    return 0;
}

