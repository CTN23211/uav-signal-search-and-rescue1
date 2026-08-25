// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  namespace
  {
    // =========================
    // Trajectory height filter
    // =========================
    // This filter is applied after a B-spline trajectory is generated/refined
    // and before updateTrajInfo(). If any sampled point violates the height
    // range, the trajectory is rejected and will not be published/executed.
    bool g_enable_traj_height_filter = true;
    double g_min_traj_z = -100.0;
    double g_max_traj_z = 1.60;
    double g_traj_height_check_dt = 0.05;
    double g_traj_height_tolerance = 0.03;  // tolerance to avoid rejecting tiny numerical overshoot

    // =========================
    // Trajectory body clearance filter
    // =========================
    // Check the 3D body envelope of the planned B-spline before updateTrajInfo().
    // It rejects routes through narrow gaps, under low obstacles, or too close to walls.
    bool g_enable_traj_body_filter = true;
    double g_traj_body_check_dt = 0.05;
    double g_traj_body_radius = 0.35;
    double g_traj_body_up_clearance = 0.50;
    double g_traj_body_down_clearance = 0.20;
    double g_traj_body_z_step = 0.10;
    int g_traj_body_circle_samples = 8;

    bool checkBsplineHeightLimit(UniformBspline &traj,
                                 double min_z,
                                 double max_z,
                                 double check_dt,
                                 double tolerance,
                                 double &min_seen_z,
                                 double &max_seen_z,
                                 double &violate_t,
                                 double &violate_z)
    {
      const double duration = traj.getTimeSum();

      min_seen_z = std::numeric_limits<double>::infinity();
      max_seen_z = -std::numeric_limits<double>::infinity();
      violate_t = 0.0;
      violate_z = 0.0;

      if (!std::isfinite(duration) || duration < 0.0)
      {
        ROS_ERROR("[HEIGHT FILTER] Invalid trajectory duration: %.6f", duration);
        return false;
      }

      if (check_dt <= 1e-4 || !std::isfinite(check_dt))
      {
        check_dt = 0.05;
      }

      const double safe_min_z = std::min(min_z, max_z);
      const double safe_max_z = std::max(min_z, max_z);
      const double safe_tol = std::max(0.0, tolerance);

      // Always check t=0. For very short trajectories, this may be the only sample.
      const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / check_dt)));

      for (int i = 0; i <= sample_num; ++i)
      {
        double t = std::min(duration, i * check_dt);
        Eigen::Vector3d p = traj.evaluateDeBoorT(t);
        const double z = p(2);

        min_seen_z = std::min(min_seen_z, z);
        max_seen_z = std::max(max_seen_z, z);

        if (!std::isfinite(z))
        {
          violate_t = t;
          violate_z = z;
          ROS_ERROR("[HEIGHT FILTER] Reject trajectory: non-finite z at t=%.3f", t);
          return false;
        }

        if (z < safe_min_z - safe_tol || z > safe_max_z + safe_tol)
        {
          violate_t = t;
          violate_z = z;
          ROS_ERROR("[HEIGHT FILTER] Reject trajectory: z=%.3f outside [%.3f, %.3f] with tol=%.3f at t=%.3f",
                    z, safe_min_z, safe_max_z, safe_tol, t);
          return false;
        }
      }

      return true;
    }

    bool checkBsplineBodyClearance(UniformBspline &traj,
                                   const GridMap::Ptr &grid_map,
                                   double check_dt,
                                   double body_radius,
                                   double up_clearance,
                                   double down_clearance,
                                   double z_step,
                                   int circle_samples,
                                   double &violate_t,
                                   Eigen::Vector3d &violate_center,
                                   Eigen::Vector3d &violate_check_pt)
    {
      const double duration = traj.getTimeSum();

      violate_t = 0.0;
      violate_center = Eigen::Vector3d::Zero();
      violate_check_pt = Eigen::Vector3d::Zero();

      if (!grid_map)
      {
        ROS_ERROR("[BODY FILTER] Invalid grid map pointer.");
        return false;
      }

      if (!std::isfinite(duration) || duration < 0.0)
      {
        ROS_ERROR("[BODY FILTER] Invalid trajectory duration: %.6f", duration);
        return false;
      }

      if (check_dt <= 1e-4 || !std::isfinite(check_dt))
        check_dt = 0.05;

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

      const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / check_dt)));

      for (int i = 0; i <= sample_num; ++i)
      {
        const double t = std::min(duration, i * check_dt);
        Eigen::Vector3d p = traj.evaluateDeBoorT(t);

        if (!std::isfinite(p(0)) || !std::isfinite(p(1)) || !std::isfinite(p(2)))
        {
          violate_t = t;
          violate_center = p;
          violate_check_pt = p;
          ROS_ERROR("[BODY FILTER] Reject trajectory: non-finite center point at t=%.3f", t);
          return false;
        }

        for (const auto &offset : offsets)
        {
          Eigen::Vector3d q = p + offset;

          if (!std::isfinite(q(0)) || !std::isfinite(q(1)) || !std::isfinite(q(2)))
          {
            violate_t = t;
            violate_center = p;
            violate_check_pt = q;
            ROS_ERROR("[BODY FILTER] Reject trajectory: non-finite envelope point at t=%.3f", t);
            return false;
          }

          if (grid_map->getInflateOccupancy(q))
          {
            violate_t = t;
            violate_center = p;
            violate_check_pt = q;

            ROS_ERROR("[BODY FILTER] Reject trajectory: 3D body envelope hits inflated obstacle at t=%.3f, "
                      "center=[%.3f %.3f %.3f], check=[%.3f %.3f %.3f], "
                      "radius=%.3f, up=%.3f, down=%.3f",
                      t,
                      p(0), p(1), p(2),
                      q(0), q(1), q(2),
                      body_radius, up_clearance, down_clearance);

            return false;
          }
        }
      }

      return true;
    }
  } // namespace


  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    // Trajectory height filter parameters.
    // Default behavior: reject any planned B-spline trajectory whose sampled z
    // is higher than 1.60 m. You can override these in advanced_param_exp.xml:
    //   manager/enable_traj_height_filter
    //   manager/min_traj_z
    //   manager/max_traj_z
    //   manager/traj_height_check_dt
    //   manager/traj_height_tolerance
    nh.param("manager/enable_traj_height_filter", g_enable_traj_height_filter, true);
    nh.param("manager/min_traj_z", g_min_traj_z, -100.0);
    nh.param("manager/max_traj_z", g_max_traj_z, 1.60);
    nh.param("manager/traj_height_check_dt", g_traj_height_check_dt, 0.05);
    nh.param("manager/traj_height_tolerance", g_traj_height_tolerance, 0.03);

    if (g_traj_height_check_dt <= 1e-4)
    {
      ROS_WARN("[HEIGHT FILTER] manager/traj_height_check_dt too small. Reset to 0.05 s.");
      g_traj_height_check_dt = 0.05;
    }

    if (g_min_traj_z > g_max_traj_z)
    {
      ROS_WARN("[HEIGHT FILTER] min_traj_z %.3f > max_traj_z %.3f. Swap them.",
               g_min_traj_z, g_max_traj_z);
      std::swap(g_min_traj_z, g_max_traj_z);
    }

    if (g_traj_height_tolerance < 0.0 || !std::isfinite(g_traj_height_tolerance))
    {
      ROS_WARN("[HEIGHT FILTER] manager/traj_height_tolerance invalid. Reset to 0.03 m.");
      g_traj_height_tolerance = 0.03;
    }

    ROS_WARN("[HEIGHT FILTER] enable=%s, z_range=[%.3f, %.3f], check_dt=%.3f s, tol=%.3f m",
             g_enable_traj_height_filter ? "true" : "false",
             g_min_traj_z, g_max_traj_z, g_traj_height_check_dt, g_traj_height_tolerance);

    // Trajectory body clearance filter parameters.
    nh.param("manager/enable_traj_body_filter", g_enable_traj_body_filter, true);
    nh.param("manager/traj_body_check_dt", g_traj_body_check_dt, 0.05);
    nh.param("manager/traj_body_radius", g_traj_body_radius, 0.35);
    nh.param("manager/traj_body_up_clearance", g_traj_body_up_clearance, 0.50);
    nh.param("manager/traj_body_down_clearance", g_traj_body_down_clearance, 0.20);
    nh.param("manager/traj_body_z_step", g_traj_body_z_step, 0.10);
    nh.param("manager/traj_body_circle_samples", g_traj_body_circle_samples, 8);

    if (g_traj_body_check_dt <= 1e-4 || !std::isfinite(g_traj_body_check_dt))
    {
      ROS_WARN("[BODY FILTER] manager/traj_body_check_dt invalid. Reset to 0.05 s.");
      g_traj_body_check_dt = 0.05;
    }

    if (g_traj_body_z_step <= 1e-4 || !std::isfinite(g_traj_body_z_step))
    {
      ROS_WARN("[BODY FILTER] manager/traj_body_z_step invalid. Reset to 0.10 m.");
      g_traj_body_z_step = 0.10;
    }

    g_traj_body_radius = std::max(0.0, g_traj_body_radius);
    g_traj_body_up_clearance = std::max(0.0, g_traj_body_up_clearance);
    g_traj_body_down_clearance = std::max(0.0, g_traj_body_down_clearance);
    g_traj_body_circle_samples = std::max(4, g_traj_body_circle_samples);

    ROS_WARN("[BODY FILTER] enable=%s, radius=%.3f, up=%.3f, down=%.3f, check_dt=%.3f, z_step=%.3f, circle_samples=%d",
             g_enable_traj_body_filter ? "true" : "false",
             g_traj_body_radius,
             g_traj_body_up_clearance,
             g_traj_body_down_clearance,
             g_traj_body_check_dt,
             g_traj_body_z_step,
             g_traj_body_circle_samples);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    ROS_WARN("[REPLAN INPUT] start_pt=[%.3f, %.3f, %.3f], start_vel=[%.3f, %.3f, %.3f], "
             "start_acc=[%.3f, %.3f, %.3f], target=[%.3f, %.3f, %.3f], target_vel=[%.3f, %.3f, %.3f]",
             start_pt(0), start_pt(1), start_pt(2),
             start_vel(0), start_vel(1), start_vel(2),
             start_acc(0), start_acc(1), start_acc(2),
             local_target_pt(0), local_target_pt(1), local_target_pt(2),
             local_target_vel(0), local_target_vel(1), local_target_vel(2));

    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (!vis_trajs.empty())
      {
        visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visualization will take up several milliseconds.
      }
      else
      {
        ROS_WARN("[VIS] No successful distinctive trajectory to display. Skip displayMultiInitPathList().");
      }
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      //static int vis_id = 0;
      if (point_set.size() >= 2)
      {
        visualization_->displayInitPathList(point_set, 0.2, 0);
      }
      else
      {
        ROS_WARN("[VIS] point_set has less than 2 points. Skip displayInitPathList().");
      }
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      ROS_ERROR("[REPLAN FAIL] Step 1 optimization failed. No new trajectory will be published.");
      // Do not display failed control points as optimal trajectory; it is misleading in RViz.
      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = ros::Time::now() - t_start;

    // =========================
    // STEP 4: HEIGHT FILTER
    // =========================
    // Reject the planned trajectory if any sampled point is outside the allowed
    // z range. This is a trajectory-level filter, not just a target-point clamp.
    if (g_enable_traj_height_filter)
    {
      double min_seen_z = 0.0;
      double max_seen_z = 0.0;
      double violate_t = 0.0;
      double violate_z = 0.0;

      if (!checkBsplineHeightLimit(pos,
                                   g_min_traj_z,
                                   g_max_traj_z,
                                   g_traj_height_check_dt,
                                   g_traj_height_tolerance,
                                   min_seen_z,
                                   max_seen_z,
                                   violate_t,
                                   violate_z))
      {
        ROS_ERROR("[HEIGHT FILTER] Planning succeeded but rejected before publishing. "
                  "z_range_seen=[%.3f, %.3f], allowed=[%.3f, %.3f], violate_t=%.3f, violate_z=%.3f",
                  min_seen_z, max_seen_z, g_min_traj_z, g_max_traj_z, violate_t, violate_z);

        // Do not display a rejected trajectory as optimal trajectory.
        // The upper FSM should handle this false return and publish a HOLD trajectory if needed.
        continous_failures_count_++;
        return false;
      }

      ROS_WARN("[HEIGHT FILTER] Trajectory accepted. z_range_seen=[%.3f, %.3f], allowed=[%.3f, %.3f], tol=%.3f",
               min_seen_z, max_seen_z, g_min_traj_z, g_max_traj_z, g_traj_height_tolerance);
    }

    // =========================
    // STEP 5: BODY CLEARANCE FILTER
    // =========================
    // Reject the planned trajectory if the 3D body envelope is too close to
    // inflated obstacles. This prevents routes through narrow gaps, under low
    // obstacles, or too close to walls even when the centerline itself is free.
    if (g_enable_traj_body_filter)
    {
      double violate_t = 0.0;
      Eigen::Vector3d violate_center = Eigen::Vector3d::Zero();
      Eigen::Vector3d violate_check_pt = Eigen::Vector3d::Zero();

      if (!checkBsplineBodyClearance(pos,
                                     grid_map_,
                                     g_traj_body_check_dt,
                                     g_traj_body_radius,
                                     g_traj_body_up_clearance,
                                     g_traj_body_down_clearance,
                                     g_traj_body_z_step,
                                     g_traj_body_circle_samples,
                                     violate_t,
                                     violate_center,
                                     violate_check_pt))
      {
        ROS_ERROR("[BODY FILTER] Planning succeeded but rejected before publishing. "
                  "violate_t=%.3f, center=[%.3f %.3f %.3f], check=[%.3f %.3f %.3f]",
                  violate_t,
                  violate_center(0), violate_center(1), violate_center(2),
                  violate_check_pt(0), violate_check_pt(1), violate_check_pt(2));

        continous_failures_count_++;
        return false;
      }

      ROS_WARN("[BODY FILTER] Trajectory accepted by 3D body clearance check.");
    }

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
