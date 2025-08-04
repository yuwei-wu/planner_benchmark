#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <api_server/replan_fsm.h>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("api_server_node");

  api_server::ReplanFSM rebo_replan;

  rebo_replan.init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}