#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::placeholders;

class NumberPublisher : public rclcpp::Node
{
    private:
        int number_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        void publishNumber()
        {
            auto msg = std_msgs::msg::Int32();
            msg.data = number_;
            publisher_->publish(msg);
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "Éxito";

            for (const auto &param : parameters) {
                if (param.get_name() == "number") {
                    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                        number_ = param.as_int();
                        RCLCPP_INFO(this->get_logger(), "Parámetro 'number' actualizado a: %d", number_);
                    } else {
                        result.successful = false;
                        result.reason = "El tipo de dato debe ser entero (int32)";
                    }
                }
            }
            return result;
        }

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

            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&NumberPublisher::parametersCallback, this, _1));
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

// Para modificar el parámetro "number" en tiempo de ejecución, se puede usar el siguiente comando en otra terminal:
// ros2 param set /number_publisher number 29