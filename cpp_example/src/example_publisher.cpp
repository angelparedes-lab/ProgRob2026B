#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherCpp : public rclcpp::Node
{

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  
  public:
    PublisherCpp() : Node("noticias_carlos2")
    {
      pub_= create_publisher<std_msgs::msg::String>("TraficoZMG",10);
      RCLCPP_INFO(get_logger(), "Noticias carlos a iniciado.");
      auto message = std_msgs::msg::String();
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