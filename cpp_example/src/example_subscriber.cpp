#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class SubscriberCpp : public rclcpp::Node
{

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  
  public:
    SubscriberCpp() : Node("Usuario_2026")
    {
      sub_= create_subscription<std_msgs::msg::String>("TraficoZMG",10,
        [this](const std_msgs::msg::String::SharedPtr msg)
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