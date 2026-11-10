#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/hardware_status.hpp"
#include "std_srvs/srv/trigger.hpp"

class MonitorNode : public rclcpp::Node {
public:
    MonitorNode() : Node("monitor_node") {
        // Declarar parámetro estático leído solo en la inicialización
        this->declare_parameter<int>("umbral_temperatura", 50);
        umbral_ = this->get_parameter("umbral_temperatura").as_int();

        RCLCPP_INFO(this->get_logger(), "Monitor iniciado. Umbral de temperatura configurado a: %ld", umbral_);

        // Suscripción al tópico
        subscription_ = this->create_subscription<custom_interfaces::msg::HardwareStatus>(
            "hardware_status", 10, std::bind(&MonitorNode::status_callback, this, std::placeholders::_1));

        // Servidor de servicio
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "check_system", std::bind(&MonitorNode::handle_check_system, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void status_callback(const custom_interfaces::msg::HardwareStatus::SharedPtr msg) {
        last_temp_ = msg->temperatura;
        last_motors_ = msg->motores_listos;

        if (last_temp_ > umbral_) {
            RCLCPP_WARN(this->get_logger(), "¡ADVERTENCIA! Temperatura actual (%ld) supera el umbral (%ld)", last_temp_, umbral_);
        }
    }

    void handle_check_system(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        if (last_temp_ <= umbral_ && last_motors_) {
            res->success = true;
            res->message = "Autorización de movimiento concedida";
        } else {
            res->success = false;
            res->message = "Bloqueo por temperatura alta o motores no listos";
        }
    }

    rclcpp::Subscription<custom_interfaces::msg::HardwareStatus>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    int64_t umbral_;
    int64_t last_temp_ = 0;
    bool last_motors_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}