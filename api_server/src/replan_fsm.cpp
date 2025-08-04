
#include <api_server/replan_fsm.h>

namespace api_server
{

  void ReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new ego_planner::PlanningVisualization(node_));

    planner_manager_.reset(new PlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&ReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&ReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    // std::bind(&ReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_msgs::msg::MultiTraj>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_msgs::msg::MultiTraj> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_msgs::msg::MultiTraj>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_msgs::msg::SingleTraj>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_msgs::msg::SingleTraj>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_msgs::msg::SingleTraj> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    bspline_pub_ = node_->create_publisher<traj_msgs::msg::SingleTraj>("planning/traj", 10);
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void ReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void ReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = next_wp;

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

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void ReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void ReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void ReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void ReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_msgs::msg::SingleTraj> &msg)
  {
      const size_t id = msg->drone_id;
      if (static_cast<int>(id) == planner_manager_->pp_.drone_id)
          return;
  
      // Time synchronization check
      rclcpp::Clock clock(RCL_SYSTEM_TIME);
      double time_diff = std::abs((clock.now() - rclcpp::Time(msg->start_time, clock.get_clock_type())).seconds());
      if (time_diff > 0.25)
      {
          RCLCPP_ERROR(node_->get_logger(), "Time diff too large! Drone ID %zu: %f s", id, time_diff);
          return;
      }
  
      // Ensure buffer capacity
      if (planner_manager_->swarm_trajs_buf_.size() <= id)
          planner_manager_->swarm_trajs_buf_.resize(id + 1, ego_planner::OneTrajDataOfSwarm{-1});
  
      const auto &bspline = msg->bsplinetraj;
      const auto &pts = bspline.pos_pts;
  
      // Compute swarm start point (De Boor start)
      Eigen::Vector3d cp0(pts[0].x, pts[0].y, pts[0].z);
      Eigen::Vector3d cp1(pts[1].x, pts[1].y, pts[1].z);
      Eigen::Vector3d cp2(pts[2].x, pts[2].y, pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4.0 * cp1 + cp2) / 6.0;
  
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0 / 3.0)
      {
          planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
          return;
      }
  
      // Convert control points and knots to Eigen structures
      const size_t pts_size = pts.size();
      const size_t knots_size = bspline.knots.size();
  
      Eigen::MatrixXd pos_pts_mat(3, pts_size);
      Eigen::VectorXd knots_vec(knots_size);
  
      for (size_t j = 0; j < pts_size; ++j)
      {
          pos_pts_mat.col(j) << pts[j].x, pts[j].y, pts[j].z;
      }
  
      for (size_t j = 0; j < knots_size; ++j)
      {
          knots_vec(j) = bspline.knots[j];
      }
  
      auto &traj_buf = planner_manager_->swarm_trajs_buf_[id];
      traj_buf.drone_id = static_cast<int>(id);
  
      // Compute trajectory duration
      double cutback = static_cast<double>(bspline.order) / 2.0 + 1.5;
      size_t floor_idx = knots_size - static_cast<size_t>(std::floor(cutback));
      size_t ceil_idx = knots_size - static_cast<size_t>(std::ceil(cutback));
  
      traj_buf.duration_ = (bspline.order % 2) 
                           ? bspline.knots[ceil_idx]
                           : (bspline.knots[floor_idx] + bspline.knots[ceil_idx]) / 2.0;
  
      // Construct B-spline trajectory
      ego_planner::UniformBspline pos_traj(pos_pts_mat, bspline.order, bspline.knots[1] - bspline.knots[0]);
      pos_traj.setKnot(knots_vec);
  
      traj_buf.position_traj_ = pos_traj;
      traj_buf.start_pos_ = pos_traj.evaluateDeBoorT(0);
      traj_buf.start_time_ = msg->start_time;
  
      // Collision Check
      if (planner_manager_->checkCollision(id))
      {
          changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
      }
  }
  





  void ReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_msgs::msg::MultiTraj> &msg)
  {
      // Clear and assign incoming trajectories
      multi_bspline_msgs_buf_ = *msg;
  
      if (!have_odom_)
      {
          RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
          return;
      }
  
      const int traj_size = static_cast<int>(msg->traj.size());
      if (traj_size != msg->drone_id_from + 1)
      {
          RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", traj_size, msg->drone_id_from + 1);
          return;
      }
  
      if (msg->traj[0].bsplinetraj.order != 3)
      {
          RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
          return;
      }
  
      // Prepare buffer
      planner_manager_->swarm_trajs_buf_.clear();
      planner_manager_->swarm_trajs_buf_.resize(traj_size);
  
      const double horizen_thresh = planning_horizen_ * 4.0 / 3.0;
  
      for (size_t i = 0; i < msg->traj.size(); ++i)
      {
          const auto &bspline = msg->traj[i].bsplinetraj;
          const auto &pos_pts = bspline.pos_pts;
          const auto &knots = bspline.knots;
  
          Eigen::Vector3d cp0(pos_pts[0].x, pos_pts[0].y, pos_pts[0].z);
          Eigen::Vector3d cp1(pos_pts[1].x, pos_pts[1].y, pos_pts[1].z);
          Eigen::Vector3d cp2(pos_pts[2].x, pos_pts[2].y, pos_pts[2].z);
  
          Eigen::Vector3d swarm_start_pt = (cp0 + 4.0 * cp1 + cp2) / 6.0;
  
          if ((swarm_start_pt - odom_pos_).norm() > horizen_thresh)
          {
              planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
              continue;
          }
  
          const size_t pts_size = pos_pts.size();
          const size_t knots_size = knots.size();
  
          Eigen::MatrixXd pos_pts_mat(3, pts_size);
          Eigen::VectorXd knots_vec(knots_size);
  
          for (size_t j = 0; j < pts_size; ++j)
          {
              pos_pts_mat(0, j) = pos_pts[j].x;
              pos_pts_mat(1, j) = pos_pts[j].y;
              pos_pts_mat(2, j) = pos_pts[j].z;
          }
  
          for (size_t j = 0; j < knots_size; ++j)
          {
              knots_vec(j) = knots[j];
          }
  
          auto &traj_buf = planner_manager_->swarm_trajs_buf_[i];
          traj_buf.drone_id = static_cast<int>(i);
  
          // Compute trajectory duration
          double cutback = static_cast<double>(bspline.order) / 2.0 + 1.5;
          if (bspline.order % 2) // Odd order
          {
              traj_buf.duration_ = knots[knots_size - static_cast<size_t>(std::ceil(cutback))];
          }
          else // Even order
          {
              double knot1 = knots[knots_size - static_cast<size_t>(std::floor(cutback))];
              double knot2 = knots[knots_size - static_cast<size_t>(std::ceil(cutback))];
              traj_buf.duration_ = (knot1 + knot2) / 2.0;
          }
  
          // Initialize uniform B-spline trajectory
          ego_planner::UniformBspline pos_traj(pos_pts_mat, bspline.order, knots[1] - knots[0]);
          pos_traj.setKnot(knots_vec);
  
          traj_buf.position_traj_ = pos_traj;
          traj_buf.start_pos_ = pos_traj.evaluateDeBoorT(0);
          traj_buf.start_time_ = msg->traj[i].start_time;
      }
  
      have_recv_pre_agent_ = true;
  }
  

  void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
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

  std::pair<int, ReplanFSM::FSM_EXEC_STATE> ReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void ReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void ReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

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
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
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
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

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
      auto *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

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

    data_disp_.header.stamp = rclcpp::Clock().now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool ReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
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
    return false;
  }

  bool ReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    auto *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = rclcpp::Clock().now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
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
          return false;
        }
      }
    }

    return true;
  }

  void ReplanFSM::checkCollisionCallback()
  {

    auto *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_cur = (rclcpp::Clock().now() - info->start_time_).seconds();

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = rclcpp::Clock().now().seconds();

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool ReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {
      auto info = &planner_manager_->local_data_;
    
      traj_msgs::msg::SingleTraj traj_msg;
      convertTraj(traj_msg);

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(traj_msg);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }


  // bool ReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  // {

  //   getLocalTarget();

  //   bool plan_and_refine_success =
  //       planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
  //   have_new_target_ = false;

  //   cout << "refine_success=" << plan_and_refine_success << endl;

  //   if (plan_and_refine_success)
  //   {

  //     auto info = &planner_manager_->local_data_;

  //     traj_utils::msg::Bspline bspline;
  //     bspline.order = 3;
  //     bspline.start_time = info->start_time_;
  //     bspline.traj_id = info->traj_id_;

  //     Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  //     bspline.pos_pts.reserve(pos_pts.cols());
  //     for (int i = 0; i < pos_pts.cols(); ++i)
  //     {
  //       geometry_msgs::msg::Point pt;
  //       pt.x = pos_pts(0, i);
  //       pt.y = pos_pts(1, i);
  //       pt.z = pos_pts(2, i);
  //       bspline.pos_pts.push_back(pt);
  //     }

  //     Eigen::VectorXd knots = info->position_traj_.getKnot();

  //     bspline.knots.reserve(knots.rows());
  //     for (int i = 0; i < knots.rows(); ++i)
  //     {
  //       bspline.knots.push_back(knots(i));
  //     }

  //     /* 1. publish traj to traj_server */
  //     bspline_pub_->publish(bspline);

  //     /* 2. publish traj to the next drone of swarm */

  //     /* 3. publish traj for visualization */
  //     visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
  //   }

  //   return plan_and_refine_success;
  // }

  void ReplanFSM::convertTraj(traj_msgs::msg::SingleTraj &traj_msg)
  {
    auto info = &planner_manager_->local_data_;
    
    traj_msg.traj_type  = traj_msgs::msg::SingleTraj::TRAJ_BSPLINE;
    traj_msg.start_time = info->start_time_;
    traj_msg.traj_id = info->traj_id_;


    traj_msg.bsplinetraj.order = 3;


    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    traj_msg.bsplinetraj.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      traj_msg.bsplinetraj.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    traj_msg.bsplinetraj.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      traj_msg.bsplinetraj.knots.push_back(knots(i));
    }

    traj_msg.duration = info->duration_;

    // rclcpp::Clock clock(RCL_SYSTEM_TIME);
    // rclcpp::Time start_time(traj_msg.start_time, clock.get_clock_type());
    // rclcpp::Time end_time(traj_msg.end_time, clock.get_clock_type());
    
    // double duration_sec = (end_time - start_time).seconds();
    
    // std::cout << "Duration: " << duration_sec << " seconds" << std::endl;

    // std::cout << "knots.rows() = " << knots.rows() << ", pos_pts.cols() = " << pos_pts.cols() << endl;
    

    return;

  }



  void ReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    traj_msgs::msg::SingleTraj traj_msg;
    convertTraj(traj_msg);

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = traj_msg;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(traj_msg);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(traj_msg);
  }

  bool ReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_msgs::msg::SingleTraj traj_msg;
    convertTraj(traj_msg);
    /* 1. publish traj to traj_server */
    bspline_pub_->publish(traj_msg);

    return true;
  }

  void ReplanFSM::getLocalTarget()
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
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
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

} // namespace api_server
