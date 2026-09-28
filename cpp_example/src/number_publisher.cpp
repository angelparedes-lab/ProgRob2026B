#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::placeholders;

class NumberPublisher : public rclcpp::Node
{
    private:

        void publishNumber()
        {
            auto msg = std_msgs::msg::Int32();
            msg.data = number_;
            publisher_->publish(msg);
        }

        int number_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        NumberPublisher() : Node("number_publisher")
        {
            this->declare_parameter("number", 2);
            this->declare_parameter("timer_period", 1.0);

            number_ = this->get_parameter("number").as_int();
            double timer_period_ = this->get_parameter("timer_period").as_double();

            publisher_ = this->create_publisher<std_msgs::msg::Int32>("number", 2);
                
            timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period_),
                std::bind(&NumberPublisher::publishNumber, this));
        }

        
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
