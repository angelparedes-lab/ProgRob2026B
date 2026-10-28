#include <chrono>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;

class HardwareEmulatorNode : public rclcpp::Node {
public:
    HardwareEmulatorNode() : Node("hardware_emulator_node"), temperatura_(45), increment_(true) {
        publisher_ = this->create_publisher<custom_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&HardwareEmulatorNode::publish_status, this));
        RCLCPP_INFO(this->get_stick(), "Nodo Emulador de Hardware iniciado.");
    }

private:
    void publish_status() {
        auto message = custom_interfaces::msg::HardwareStatus();
        
        // Simular variación de temperatura
        if (increment_) {
            temperatura_ += 3;
            if (temperatura_ >= 80) increment_ = false;
        } else {
            temperatura_ -= 3;
            if (temperatura_ <= 40) increment_ = true;
        }

        message.temperatura = temperatura_;
        message.motores_listos = (temperatura_ < 75);

        if (temperatura_ > 75) {
            message.debug_msg = "Fallo de sensor / Temperatura critica";
        } else if (temperatura_ > 60) {
            message.debug_msg = "Calentando";
        } else {
            message.debug_msg = "Operacion normal";
        }

        publisher_->publish(message);
    }

    rclcpp::Publisher<custom_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int64_t temperatura_;
    bool increment_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareEmulatorNode>());
    rclcpp::shutdown();
    return 0;
}