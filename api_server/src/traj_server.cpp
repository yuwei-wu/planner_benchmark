#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include <poly_lib/ros_utils.hpp>
#include <traj_msgs/msg/single_traj.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};



using namespace std;

bool receive_traj_ = false;
traj_opt::TrajData pre_traj_, traj_;

double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;


void uniformTrajCallback(const traj_msgs::msg::SingleTraj::SharedPtr msg)
{
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  // Duration in seconds as double

  traj_ = traj_opt::TrajData();  // Resets all members to default values
  SingleTraj2TrajData(msg, traj_);
  traj_duration_ = traj_.traj_dur_;


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