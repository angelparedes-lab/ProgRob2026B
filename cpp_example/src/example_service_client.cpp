#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

class ServiceClientCpp : public rclcpp::Node
{
  private:
    std::vector<std::thread> threads_;
  public:
    ServiceClientCpp() : Node("add_two_ints_client")
    {
        threads_.push_back(std::thread(&ServiceClientCpp::callAddTwoIntsService, this, 1, 2));
        threads_.push_back(std::thread(&ServiceClientCpp::callAddTwoIntsService, this, 3, 4));
        threads_.push_back(std::thread(&ServiceClientCpp::callAddTwoIntsService, this, 5, 6));
        threads_.push_back(std::thread(&ServiceClientCpp::callAddTwoIntsService, this, 7, 8));

    }

    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Esperando al servidor");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int) a, (int) b, 
                    (int) response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error al conectar con el servicio");
        }
        
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceClientCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}