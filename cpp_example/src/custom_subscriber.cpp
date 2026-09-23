#include <rclcpp/rclcpp.hpp>
#include "robx7_interfaces/msg/custom_string.hpp"


class SubscriberCpp : public rclcpp::Node
{

  private:
    rclcpp::Subscription<robx7_interfaces::msg::CustomString>::SharedPtr sub_;
  
  public:
    SubscriberCpp() : Node("Usuario_2026")
    {
      sub_= create_subscription<robx7_interfaces::msg::CustomString>("TraficoZMG",10,
        [this](const robx7_interfaces::msg::CustomString::SharedPtr msg)
        {
          RCLCPP_INFO(get_logger(), "Recibido: '%s'", msg->data.c_str());
        }
      );
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}