#include <rclcpp/rclcpp.hpp>
#include "robx7_interfaces/srv/sumar_dos_flotantes.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ServiceServerCpp : public rclcpp::Node
{
  private:
    rclcpp::Service<robx7_interfaces::srv::SumarDosFlotantes>::SharedPtr server_;
    void callbackAddTwoInts(const robx7_interfaces::srv::SumarDosFlotantes::Request::SharedPtr request,
                            const robx7_interfaces::srv::SumarDosFlotantes::Response::SharedPtr response)
    {
        response->suma = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%f + %f = %f", (float) request->a, (float) request->b, 
                    (float) response->suma);
    }
  
  public:
    ServiceServerCpp() : Node("add_two_ints_server")
    {
        server_ = this->create_service<robx7_interfaces::srv::SumarDosFlotantes>("add_two_ints",
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