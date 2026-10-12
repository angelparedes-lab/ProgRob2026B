#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class OperatorNode : public rclcpp::Node {
public:
    OperatorNode() : Node("operator_node") {
        client_ = this->create_client<std_srvs::srv::Trigger>("check_system");
        timer_ = this->create_wall_timer(3s, std::bind(&OperatorNode::send_request, this));
    }

private:
    void send_request() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Servicio no disponible, esperando...");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Respuesta del Servidor: [EXITO] %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Respuesta del Servidor: [BLOQUEO] %s", response->message.c_str());
            }
        });
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OperatorNode>());
    rclcpp::shutdown();
    return 0;
}