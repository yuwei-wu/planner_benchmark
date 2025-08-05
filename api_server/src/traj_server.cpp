#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include <poly_lib/traj_data.hpp>
#include <traj_msgs/msg/single_traj.hpp>


rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

// traj data
struct TrajData
{
  /* info of generated traj */
  double traj_dur_ = 0, traj_yaw_dur_ = 0;
  rclcpp::Time start_time_;
  int dim_;

  traj_opt::Trajectory3D traj_3d_;
  traj_opt::Trajectory1D traj_yaw_;
  traj_opt::DiscreteStates traj_discrete_;
};

using namespace std;

bool receive_traj_ = false;
TrajData pre_traj_, traj_;

double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;


void uniformTrajCallback(const traj_msgs::msg::SingleTraj::SharedPtr msg)
{
  // # === Identification Info ===
  // int32 drone_id                    # Unique ID of the drone
  // int32 traj_id                     # Unique ID of the trajectory instance
  // builtin_interfaces/Time start_time                # Trajectory start time (seconds, relative or absolute)
  // builtin_interfaces/Time end_time                  # Trajectory end time (seconds, relative or absolute)
  
  // # === Trajectory Type ===
  // int32 TRAJ_POLYNOMIAL = 0
  // int32 TRAJ_BSPLINE = 1
  // int32 TRAJ_DISCRETE = 2
  // int32 TRAJ_BD_DERIVATIVE = 3
  
  // int32 traj_type                   # Specifies which trajectory type is active in this message
  
  // # === Trajectory Representations ===
  // Polynomial polytraj               # Polynomial parameterization (active if traj_type == TRAJ_POLYNOMIAL)
  // Bspline bsplinetraj               # B-spline parameterization (active if traj_type == TRAJ_BSPLINE)
  // Discrete discretetraj             # Discrete state-control trajectory (active if traj_type == TRAJ_DISCRETE)
  // BdDervi bddervitraj               # Boundary Derivatives (active if traj_type == TRAJ_BD_DERIVATIVE)
  start_time_ = msg->start_time;

  // Duration in seconds as double
  traj_duration_ = msg->duration;
  traj_id_ = msg->traj_id;
  traj_ = TrajData();  // Resets all members to default values

  std::vector<traj_opt::Piece<3>> segs_3d;

  switch (msg->traj_type)
  {
  case traj_msgs::msg::SingleTraj::TRAJ_POLY:
    {
      for(size_t i = 0; i < msg->polytraj.seg_x.size(); ++i)
      {
        Eigen::MatrixXd Coeffs(3, 6);
        float dt = msg->polytraj.seg_x[i].dt;
        traj_opt::Piece<3> seg(traj_opt::STANDARD, Coeffs, dt);
        for(size_t j = 0; j < 6; ++j)
        {
          Coeffs(0, j) = msg->polytraj.seg_x[i].coeffs[j];
          Coeffs(1, j) = msg->polytraj.seg_y[i].coeffs[j];
          Coeffs(2, j) = msg->polytraj.seg_z[i].coeffs[j];
        }
        segs_3d.push_back(seg);
      }
      traj_.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj_duration_);
      break;
    }
  case traj_msgs::msg::SingleTraj::TRAJ_BSPLINE:
    {
      std::cout << "Received B-spline trajectory with " << msg->bsplinetraj.pos_pts.size() << " control points." << std::endl;
      size_t N = msg->bsplinetraj.pos_pts.size() - 1;
      size_t M = msg->bsplinetraj.knots.size();
      size_t degree = M - N - 1;

      std::cout << "B-spline degree: " << degree << std::endl;

      Eigen::MatrixXd pos_pts(N + 1, 3);  // N + 1
      Eigen::VectorXd knots(M);              // N + degree + 1
      for(size_t i = 0; i < M; ++i)
      {
        knots(i) = msg->bsplinetraj.knots[i];
      }
      for(size_t i = 0; i <= N; ++i)
      {
        pos_pts(i, 0) = msg->bsplinetraj.pos_pts[i].x;
        pos_pts(i, 1) = msg->bsplinetraj.pos_pts[i].y;
        pos_pts(i, 2) = msg->bsplinetraj.pos_pts[i].z;
      }

      for(size_t i = 0; i < M - 2 * degree; i++)
      {
        Eigen::MatrixXd cpts(degree + 1, 3);
        
        for(size_t j = 0; j <= degree; j++)
        {
          cpts.row(j) = pos_pts.row(i + j);
        }

        double dt = knots(degree + i + 1) - knots(degree + i);
        traj_opt::Piece<3> seg(traj_opt::BEZIER, cpts, dt, degree);
        segs_3d.push_back(seg);
      }

      std::cout << "Total segments in B-spline trajectory: " << segs_3d.size() << std::endl;
      traj_.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj_duration_);
      break;
    }
  case traj_msgs::msg::SingleTraj::TRAJ_DISCRETE:
    {
      std::vector<Eigen::VectorXd> states;
      auto distraj = msg->discretetraj;
      size_t N = distraj.pos_pts.size();
      double dt = distraj.dt;
      for(size_t j = 0; j < N; ++j)
      {
        Eigen::VectorXd s(9);

        s  << distraj.pos_pts[j].x, distraj.pos_pts[j].y, distraj.pos_pts[j].z,
              distraj.vel_pts[j].x, distraj.vel_pts[j].y, distraj.vel_pts[j].z,
              distraj.acc_pts[j].x, distraj.acc_pts[j].y, distraj.acc_pts[j].z;
        states.push_back(s);
      }
      traj_.traj_discrete_ = traj_opt::DiscreteStates(dt, N, states);
      std::cout << "Received discrete trajectory with " << N << " states." << std::endl;
      break;
    }

  case traj_msgs::msg::SingleTraj::TRAJ_BD_DERIV:
    {
      std::cout << "Received boundary derivative trajectory." << std::endl;

      size_t N = msg->bddervitraj.durations.size();

      // Helper lambda to convert a geometry_msgs::Point/Vector3 to Eigen::Vector3d
      auto toEigen = [](const auto& v) {
        return Eigen::Vector3d(v.x, v.y, v.z);
      };
      // Pre-extract references for brevity
      const auto& start_pos = msg->bddervitraj.start_pos;
      const auto& start_vel = msg->bddervitraj.start_vel;
      const auto& start_acc = msg->bddervitraj.start_acc;
      const auto& end_pos   = msg->bddervitraj.end_pos;
      const auto& end_vel   = msg->bddervitraj.end_vel;
      const auto& end_acc   = msg->bddervitraj.end_acc;

      const auto& inner_pos = msg->bddervitraj.inner_pos;
      const auto& inner_vel = msg->bddervitraj.inner_vel;
      const auto& inner_acc = msg->bddervitraj.inner_acc;

      for (size_t i = 0; i < N; ++i)
      {
        Eigen::MatrixXd boundCond(3, 6);

        // Starting boundary conditions
        if (i == 0) {
          boundCond.col(0) = toEigen(start_pos);
          boundCond.col(1) = toEigen(start_vel);
          boundCond.col(2) = toEigen(start_acc);
        } else {
          boundCond.col(0) = toEigen(inner_pos[i - 1]);
          boundCond.col(1) = toEigen(inner_vel[i - 1]);
          boundCond.col(2) = toEigen(inner_acc[i - 1]);
        }

        // Ending boundary conditions
        if (i == N - 1) {
          boundCond.col(3) = toEigen(end_pos);
          boundCond.col(4) = toEigen(end_vel);
          boundCond.col(5) = toEigen(end_acc);
        } else {
          boundCond.col(3) = toEigen(inner_pos[i]);
          boundCond.col(4) = toEigen(inner_vel[i]);
          boundCond.col(5) = toEigen(inner_acc[i]);
        }

        double dt = msg->bddervitraj.durations[i];
        traj_opt::Piece<3> seg(traj_opt::BOUNDARY, boundCond, dt);
        segs_3d.push_back(seg);
      }
      traj_.traj_3d_ = traj_opt::Trajectory3D(segs_3d, traj_duration_);
      break;
    }
  default:
    {
      RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "Unknown trajectory type: %d", msg->traj_type);
      return;
    }
  }

  receive_traj_ = true;
  pre_traj_ = traj_;

}


// Helper function to normalize angle to [-PI, PI]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Helper function to clamp value between min and max
inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

std::pair<double, double> calculate_yaw(double t_cur, const Eigen::Vector3d& pos, 
                                       const rclcpp::Time& time_now, const rclcpp::Time& time_last)
{
    // Constants
    static constexpr double PI = M_PI;
    static constexpr double YAW_DOT_MAX_PER_SEC = PI;
    static constexpr double MIN_DIR_MAGNITUDE = 0.1;
    static constexpr double LPF_ALPHA = 0.5;  // Low-pass filter coefficient
    
    // Calculate time step (with safety check)
    const double dt = std::max(1e-6, (time_now - time_last).seconds());
    const double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
    
    // Calculate direction vector for desired yaw
    const double look_ahead_time = std::min(t_cur + time_forward_, traj_duration_);
    const Eigen::Vector3d dir = pre_traj_.traj_3d_.getPos(look_ahead_time) - pos;
    
    // Calculate target yaw angle
    const double yaw_target = (dir.norm() > MIN_DIR_MAGNITUDE) ? 
                             atan2(dir.y(), dir.x()) : last_yaw_;
    
    // Calculate shortest angular distance between target and current yaw
    double yaw_error = normalizeAngle(yaw_target - last_yaw_);
    
    // Apply rate limiting to yaw change
    const double limited_yaw_change = clamp(yaw_error, -max_yaw_change, max_yaw_change);
    double yaw = normalizeAngle(last_yaw_ + limited_yaw_change);
    
    // Calculate yaw rate
    double yawdot;
    if (std::abs(limited_yaw_change) >= max_yaw_change - 1e-6) {
        // Rate limited case
        yawdot = (limited_yaw_change > 0) ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
    } else {
        // Normal case - calculate actual rate
        yawdot = limited_yaw_change / dt;
    }
    
    // Apply low-pass filtering for smoother motion
    if (std::abs(yaw - last_yaw_) <= max_yaw_change) {
        yaw = LPF_ALPHA * last_yaw_ + (1.0 - LPF_ALPHA) * yaw;
    }
    yawdot = LPF_ALPHA * last_yaw_dot_ + (1.0 - LPF_ALPHA) * yawdot;
    
    // Update state variables
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
    
    return {yaw, yawdot};
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // clock time sync
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);
  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = pre_traj_.traj_3d_.getPos(t_cur);
    vel = pre_traj_.traj_3d_.getVel(t_cur);
    acc = pre_traj_.traj_3d_.getAcc(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);

    // final position
    pos_f = traj_.traj_3d_.getPos(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = pre_traj_.traj_3d_.getPos(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;


  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  auto traj_sub = node->create_subscription<traj_msgs::msg::SingleTraj>(
      "planning/traj",
      10,
      uniformTrajCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}