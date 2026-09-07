#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_waypoint_controller/odom_listener.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtlebot3_waypoint_controller::OdomListener>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
