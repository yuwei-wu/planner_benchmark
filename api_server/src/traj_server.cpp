#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include <poly_lib/ros_utils.hpp>
#include <traj_msgs/msg/single_traj.hpp>

using namespace std;

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
quadrotor_msgs::msg::PositionCommand cmd;

double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
traj_opt::TrajData pre_traj_, traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_ = 0;

double last_yaw_ = 0.0, last_yaw_dot_ = 0.0;
double time_forward_ = -1.0;

inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

void uniformTrajCallback(const traj_msgs::msg::SingleTraj::SharedPtr msg)
{
    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_ = traj_opt::TrajData();  // Reset
    SingleTraj2TrajData(msg, traj_);

    
    traj_duration_ = traj_.traj_dur_;
    pre_traj_ = traj_;
    receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, const Eigen::Vector3d& pos, 
                                       const rclcpp::Time& time_now, const rclcpp::Time& time_last)
{
    static constexpr double PI = M_PI;
    static constexpr double YAW_DOT_MAX_PER_SEC = PI;
    static constexpr double MIN_DIR_MAGNITUDE = 0.1;
    static constexpr double LPF_ALPHA = 0.5;

    double dt = std::max(1e-6, (time_now - time_last).seconds());
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;

    double look_ahead_time = std::min(t_cur + time_forward_, traj_duration_);
    Eigen::Vector3d dir = pre_traj_.traj_3d_.getPos(look_ahead_time) - pos;

    double yaw_target = (dir.norm() > MIN_DIR_MAGNITUDE) ? atan2(dir.y(), dir.x()) : last_yaw_;
    double yaw_error = normalizeAngle(yaw_target - last_yaw_);
    double limited_yaw_change = clamp(yaw_error, -max_yaw_change, max_yaw_change);
    double yaw = normalizeAngle(last_yaw_ + limited_yaw_change);

    double yawdot = (std::abs(limited_yaw_change) >= max_yaw_change - 1e-6) ? 
                    copysign(YAW_DOT_MAX_PER_SEC, limited_yaw_change) : limited_yaw_change / dt;

    // Low-pass filter for smoothness
    yaw = LPF_ALPHA * last_yaw_ + (1.0 - LPF_ALPHA) * yaw;
    yawdot = LPF_ALPHA * last_yaw_dot_ + (1.0 - LPF_ALPHA) * yawdot;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    return {yaw, yawdot};
}

void cmdCallback()
{
    if (!receive_traj_) return;

    static rclcpp::Clock clock(RCL_ROS_TIME);
    static rclcpp::Time time_last = clock.now();

    rclcpp::Time time_now = clock.now();
    double t_cur = (time_now - start_time_).seconds();

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos_f;
    std::pair<double, double> yaw_yawdot(0, 0);

    if (t_cur >= 0.0 && t_cur < traj_duration_) {
        pos = pre_traj_.traj_3d_.getPos(t_cur);
        vel = pre_traj_.traj_3d_.getVel(t_cur);
        acc = pre_traj_.traj_3d_.getAcc(t_cur);
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);

        double tf = std::min(traj_duration_, t_cur + 2.0);
        pos_f = traj_.traj_3d_.getPos(tf);
    } 
    else if (t_cur >= traj_duration_) {
        pos = pre_traj_.traj_3d_.getPos(traj_duration_);
        pos_f = pos;
        vel.setZero();
        acc.setZero();
        yaw_yawdot = {last_yaw_, 0};
    } 
    else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("traj_server"), "[Traj server]: invalid time.");
        return;
    }

    time_last = time_now;

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();

    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();

    cmd.acceleration.x = acc.x();
    cmd.acceleration.y = acc.y();
    cmd.acceleration.z = acc.z();

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("traj_server");

    node->declare_parameter("traj_server/time_forward", -1.0);
    node->get_parameter("traj_server/time_forward", time_forward_);

    pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
    auto traj_sub = node->create_subscription<traj_msgs::msg::SingleTraj>(
        "planning/traj", 10, uniformTrajCallback);

    auto cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);

    // Initialize control parameters
    std::copy(begin(pos_gain), end(pos_gain), cmd.kx.begin());
    std::copy(begin(vel_gain), end(vel_gain), cmd.kv.begin());

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
