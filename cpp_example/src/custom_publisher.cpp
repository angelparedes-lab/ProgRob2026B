#include <rclcpp/rclcpp.hpp>
#include "robx7_interfaces/msg/custom_string.hpp"

class PublisherCpp : public rclcpp::Node
{

  private:
    rclcpp::Publisher<robx7_interfaces::msg::CustomString>::SharedPtr pub_;
  
  public:
    PublisherCpp() : Node("noticias_carlos2")
    {
      pub_= create_publisher<robx7_interfaces::msg::CustomString>("TraficoZMG",10);
      RCLCPP_INFO(get_logger(), "Noticias carlos a iniciado.");
      auto message = robx7_interfaces::msg::CustomString();
      message.data = "Noticias de trafico de carlos";
      pub_->publish(message);
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}