#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("carlos");
  RCLCPP_INFO(node->get_logger(), "Test node carlos has been started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}