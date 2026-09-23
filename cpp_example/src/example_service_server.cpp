#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class ServiceServerCpp : public rclcpp::Node
{
  private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int) request->a, (int) request->b, 
                    (int) response->sum);
    }
  
  public:
    ServiceServerCpp() : Node("add_two_ints_server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
            std::bind(&ServiceServerCpp::callbackAddTwoInts, this,_1,_2));
      
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceServerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}