#include <rclcpp/rclcpp.hpp>
#include "rhphumanoid_walking/rhphumanoid_walking_module.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rhp::WalkingModule>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
