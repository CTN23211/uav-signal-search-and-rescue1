
#include <plan_manage/ego_replan_fsm.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ego_planner
{

  namespace
  {
    // =========================
    // Real-flight failsafe parameters
    // =========================
    // If EGO fails to find a valid trajectory, publish a HOLD trajectory at
    // the current odometry position instead of allowing the old trajectory
    // to continue driving the vehicle. These are file-local parameters so
    // no header modification is required.
    bool g_enable_hold_on_replan_failure = true;
    double g_hold_replan_min_interval = 0.20;
    ros::Time g_last_hold_pub_time;

    // Clamp manual 3D goal height before it enters the global trajectory.
    bool g_enable_target_z_clamp = true;
    double g_target_z_default = 1.0;
    double g_min_target_z = 0.30;
    double g_max_target_z = 1.60;

    // In real flight, using the old B-spline point as the next planning start
    // can keep the planner biased toward a previous wrong direction. This
    // option lets replanning start from the actual odometry state.
    bool g_replan_start_from_odom = true;
    double g_replan_odom_error_threshold = 0.30;

    // =========================
    // Online future trajectory 3D body-envelope check
    // =========================
    // This checks the trajectory that is currently being executed. If the
    // future body envelope of the vehicle will hit an inflated obstacle within
    // the configured time horizon, the FSM replans immediately.
    bool g_enable_future_body_check = true;
    double g_future_check_horizon_time = 3.0;
    double g_future_body_check_dt = 0.05;
    double g_future_body_radius = 0.35;
    double g_future_body_up_clearance = 0.50;
    double g_future_body_down_clearance = 0.20;
    double g_future_body_z_step = 0.10;
    int g_future_body_circle_samples = 8;

    double clampDouble(double value, double low, double high)
    {
      if (low > high)
        std::swap(low, high);
      return std::max(low, std::min(high, value));
    }

    bool checkPointBodyOccupied(const GridMap::Ptr &map,
                                const Eigen::Vector3d &center,
                                double body_radius,
                                double up_clearance,
                                double down_clearance,
                                double z_step,
                                int circle_samples,
                                Eigen::Vector3d &hit_point)
    {
      hit_point = center;

      if (!map)
      {
        ROS_ERROR("[FUTURE BODY CHECK] Invalid grid map pointer.");
        return true;
      }

      if (!std::isfinite(center(0)) || !std::isfinite(center(1)) || !std::isfinite(center(2)))
      {
        ROS_ERROR("[FUTURE BODY CHECK] Non-finite trajectory center point.");
        return true;
      }

      body_radius = std::max(0.0, body_radius);
      up_clearance = std::max(0.0, up_clearance);
      down_clearance = std::max(0.0, down_clearance);

      if (z_step <= 1e-4 || !std::isfinite(z_step))
        z_step = 0.10;

      if (circle_samples < 4)
        circle_samples = 4;

      std::vector<Eigen::Vector3d> offsets;
      offsets.reserve(256);

      // Center vertical column: prevents passing under low obstacles.
      for (double dz = -down_clearance; dz <= up_clearance + 1e-6; dz += z_step)
      {
        offsets.push_back(Eigen::Vector3d(0.0, 0.0, dz));
      }

      // Cylindrical envelope: prevents narrow-gap and wall-scraping trajectories.
      constexpr double kTwoPi = 6.28318530717958647692;
      for (double dz = -down_clearance; dz <= up_clearance + 1e-6; dz += z_step)
      {
        for (int k = 0; k < circle_samples; ++k)
        {
          const double a = kTwoPi * static_cast<double>(k) / static_cast<double>(circle_samples);
          offsets.push_back(Eigen::Vector3d(body_radius * std::cos(a),
                                            body_radius * std::sin(a),
                                            dz));
        }
      }

      for (const auto &offset : offsets)
      {
        Eigen::Vector3d q = center + offset;

        if (!std::isfinite(q(0)) || !std::isfinite(q(1)) || !std::isfinite(q(2)))
        {
          hit_point = q;
          ROS_ERROR("[FUTURE BODY CHECK] Non-finite body-envelope check point.");
          return true;
        }

        if (map->getInflateOccupancy(q))
        {
          hit_point = q;
          return true;
        }
      }

      return false;
    }
  } // namespace

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    // Real-flight failsafe parameters.
    nh.param("fsm/enable_hold_on_replan_failure", g_enable_hold_on_replan_failure, true);
    nh.param("fsm/hold_replan_min_interval", g_hold_replan_min_interval, 0.20);
    nh.param("fsm/enable_target_z_clamp", g_enable_target_z_clamp, true);
    nh.param("fsm/target_z_default", g_target_z_default, 1.0);
    nh.param("fsm/min_target_z", g_min_target_z, 0.30);
    nh.param("fsm/max_target_z", g_max_target_z, 1.60);
    nh.param("fsm/replan_start_from_odom", g_replan_start_from_odom, true);
    nh.param("fsm/replan_odom_error_threshold", g_replan_odom_error_threshold, 0.30);

    // Online future body-envelope safety check parameters.
    nh.param("fsm/enable_future_body_check", g_enable_future_body_check, true);
    nh.param("fsm/future_check_horizon_time", g_future_check_horizon_time, 3.0);
    nh.param("fsm/future_body_check_dt", g_future_body_check_dt, 0.05);
    nh.param("fsm/future_body_radius", g_future_body_radius, 0.35);
    nh.param("fsm/future_body_up_clearance", g_future_body_up_clearance, 0.50);
    nh.param("fsm/future_body_down_clearance", g_future_body_down_clearance, 0.20);
    nh.param("fsm/future_body_z_step", g_future_body_z_step, 0.10);
    nh.param("fsm/future_body_circle_samples", g_future_body_circle_samples, 8);

    if (g_hold_replan_min_interval < 0.0)
      g_hold_replan_min_interval = 0.0;

    if (g_min_target_z > g_max_target_z)
    {
      ROS_WARN("[TARGET Z LIMIT] min_target_z %.3f > max_target_z %.3f. Swap them.",
               g_min_target_z, g_max_target_z);
      std::swap(g_min_target_z, g_max_target_z);
    }

    ROS_WARN("[REPLAN FAILSAFE] hold_on_fail=%s, min_interval=%.2f s, replan_start_from_odom=%s, odom_err_thr=%.2f m",
             g_enable_hold_on_replan_failure ? "true" : "false",
             g_hold_replan_min_interval,
             g_replan_start_from_odom ? "true" : "false",
             g_replan_odom_error_threshold);

    if (g_future_check_horizon_time < 0.0 || !std::isfinite(g_future_check_horizon_time))
      g_future_check_horizon_time = 3.0;

    if (g_future_body_check_dt <= 1e-4 || !std::isfinite(g_future_body_check_dt))
      g_future_body_check_dt = 0.05;

    if (g_future_body_z_step <= 1e-4 || !std::isfinite(g_future_body_z_step))
      g_future_body_z_step = 0.10;

    g_future_body_radius = std::max(0.0, g_future_body_radius);
    g_future_body_up_clearance = std::max(0.0, g_future_body_up_clearance);
    g_future_body_down_clearance = std::max(0.0, g_future_body_down_clearance);
    g_future_body_circle_samples = std::max(4, g_future_body_circle_samples);

    ROS_WARN("[FUTURE BODY CHECK] enable=%s, horizon=%.2f s, dt=%.3f s, radius=%.3f, up=%.3f, down=%.3f, z_step=%.3f, circle_samples=%d",
             g_enable_future_body_check ? "true" : "false",
             g_future_check_horizon_time,
             g_future_body_check_dt,
             g_future_body_radius,
             g_future_body_up_clearance,
             g_future_body_down_clearance,
             g_future_body_z_step,
             g_future_body_circle_samples);

    ROS_WARN("[TARGET Z LIMIT] enable=%s, default_z=%.2f, range=[%.2f, %.2f]",
             g_enable_target_z_clamp ? "true" : "false",
             g_target_z_default,
             g_min_target_z,
             g_max_target_z);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    double target_z = msg->pose.position.z;
    if (g_enable_target_z_clamp)
    {
      if (target_z < 0.2)
      {
        ROS_WARN("[TARGET Z LIMIT] Received z=%.3f, use default_z=%.3f.",
                 target_z, g_target_z_default);
        target_z = g_target_z_default;
      }

      const double unclamped_z = target_z;
      target_z = clampDouble(target_z, g_min_target_z, g_max_target_z);
      if (std::fabs(target_z - unclamped_z) > 1e-6)
      {
        ROS_WARN("[TARGET Z LIMIT] Clamp target z from %.3f to %.3f, allowed=[%.3f, %.3f].",
                 unclamped_z, target_z, g_min_target_z, g_max_target_z);
      }
    }

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, target_z);

    ROS_WARN("[MANUAL TARGET] frame=%s, target=[%.3f, %.3f, %.3f]",
             msg->header.frame_id.c_str(), end_wp(0), end_wp(1), end_wp(2));

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }

    if (g_enable_hold_on_replan_failure && have_odom_)
    {
      const ros::Time now = ros::Time::now();
      if (g_last_hold_pub_time.isZero() ||
          (now - g_last_hold_pub_time).toSec() >= g_hold_replan_min_interval)
      {
        ROS_ERROR("[REPLAN FAILSAFE] planFromGlobalTraj failed after %d trial(s). Hold current odom position [%.3f, %.3f, %.3f].",
                  trial_times, odom_pos_(0), odom_pos_(1), odom_pos_(2));
        callEmergencyStop(odom_pos_);
        g_last_hold_pub_time = now;
      }
    }

    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // Keep the old EGO behavior available, but make real-flight replanning
    // safer: if the old trajectory point is too far from odometry, or if the
    // parameter asks for it, start replanning from current odom instead.
    if (info->duration_ > 1e-5)
    {
      t_cur = std::max(0.0, std::min(info->duration_, t_cur));
    }
    else
    {
      t_cur = 0.0;
    }

    Eigen::Vector3d traj_start_pt = info->position_traj_.evaluateDeBoorT(t_cur);
    Eigen::Vector3d traj_start_vel = info->velocity_traj_.evaluateDeBoorT(t_cur);
    Eigen::Vector3d traj_start_acc = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const double odom_traj_err = (traj_start_pt - odom_pos_).norm();
    const bool use_odom_start = g_replan_start_from_odom ||
                                odom_traj_err > g_replan_odom_error_threshold;

    if (use_odom_start)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      ROS_WARN("[REPLAN START] Use odom as start. odom=[%.3f, %.3f, %.3f], traj_pt=[%.3f, %.3f, %.3f], err=%.3f",
               odom_pos_(0), odom_pos_(1), odom_pos_(2),
               traj_start_pt(0), traj_start_pt(1), traj_start_pt(2),
               odom_traj_err);
    }
    else
    {
      start_pt_ = traj_start_pt;
      start_vel_ = traj_start_vel;
      start_acc_ = traj_start_acc;

      ROS_WARN("[REPLAN START] Use current B-spline as start. start=[%.3f, %.3f, %.3f], odom_err=%.3f",
               start_pt_(0), start_pt_(1), start_pt_(2), odom_traj_err);
    }

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          if (g_enable_hold_on_replan_failure && have_odom_)
          {
            const ros::Time now = ros::Time::now();
            if (g_last_hold_pub_time.isZero() ||
                (now - g_last_hold_pub_time).toSec() >= g_hold_replan_min_interval)
            {
              ROS_ERROR("[REPLAN FAILSAFE] planFromCurrentTraj failed after all attempts. Hold current odom position [%.3f, %.3f, %.3f].",
                        odom_pos_(0), odom_pos_(1), odom_pos_(2));
              callEmergencyStop(odom_pos_);
              g_last_hold_pub_time = now;
            }
          }
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    if (!map)
    {
      ROS_ERROR("[SAFETY] grid_map_ is null. Skip collision check.");
      return;
    }

    if (info->duration_ <= 1e-4)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }

    /* ---------- check future executing trajectory with 3D body envelope ---------- */
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    t_cur = std::max(0.0, std::min(info->duration_, t_cur));

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);

    double time_step = g_enable_future_body_check ? g_future_body_check_dt : 0.01;
    if (time_step <= 1e-4 || !std::isfinite(time_step))
      time_step = 0.05;

    const double check_horizon = g_enable_future_body_check ? g_future_check_horizon_time : info->duration_;
    const double t_check_end = std::min(info->duration_, t_cur + std::max(0.0, check_horizon));

    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    const double t_cur_global = ros::Time::now().toSec();

    for (double t = t_cur; t <= t_check_end + 1e-6; t += time_step)
    {
      Eigen::Vector3d p = info->position_traj_.evaluateDeBoorT(t);

      bool occ = false;
      Eigen::Vector3d hit_p = p;

      if (g_enable_future_body_check)
      {
        occ = checkPointBodyOccupied(map,
                                     p,
                                     g_future_body_radius,
                                     g_future_body_up_clearance,
                                     g_future_body_down_clearance,
                                     g_future_body_z_step,
                                     g_future_body_circle_samples,
                                     hit_p);
      }
      else
      {
        occ = map->getInflateOccupancy(p);
        hit_p = p;
      }

      // Keep the original swarm safety check.
      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) ||
            (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();

        // The received swarm trajectory may be stale or outside its duration.
        if (t_X < 0.0)
          continue;

        if (planner_manager_->swarm_trajs_buf_.at(id).duration_ > 1e-4)
          t_X = std::min(t_X, planner_manager_->swarm_trajs_buf_.at(id).duration_);

        Eigen::Vector3d swarm_predicted =
            planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);

        double dist = (p_cur - swarm_predicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          hit_p = swarm_predicted;
          break;
        }
      }

      if (occ)
      {
        const double time_to_collision = std::max(0.0, t - t_cur);

        ROS_ERROR("[FUTURE BODY CHECK] Current trajectory unsafe in %.2f s. "
                  "center=[%.3f %.3f %.3f], hit=[%.3f %.3f %.3f]",
                  time_to_collision,
                  p(0), p(1), p(2),
                  hit_p(0), hit_p(1), hit_p(2));

        if (planFromCurrentTraj())
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY_BODY_REPLAN");
          publishSwarmTrajs(false);
          return;
        }

        if (time_to_collision < emergency_time_)
        {
          ROS_ERROR("[FUTURE BODY CHECK] Unsafe trajectory is too close. Emergency stop.");
          changeFSMExecState(EMERGENCY_STOP, "SAFETY_BODY");
        }
        else
        {
          ROS_WARN("[FUTURE BODY CHECK] Replan failed. Switch to REPLAN_TRAJ and keep HOLD/retry.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY_BODY");
        }

        return;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (!plan_and_refine_success)
    {
      ROS_ERROR("[REPLAN RESULT] reboundReplan failed. No new normal trajectory will be published by callReboundReplan().");
    }

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    ROS_ERROR("[EMERGENCY HOLD] Published HOLD trajectory at [%.3f, %.3f, %.3f], traj_id=%d.",
              stop_pos(0), stop_pos(1), stop_pos(2), info->traj_id_);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important conor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape conor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
