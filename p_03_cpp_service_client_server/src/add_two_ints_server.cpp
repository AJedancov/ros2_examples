#include "rclcpp/rclcpp.hpp"
#include "p_01_custom_interfaces/srv/add_two_ints.hpp"

#include <memory> // Dynamic Memory management library

class ServiceServer: public rclcpp::Node{
public:
  ServiceServer(const std::string nodeName): rclcpp::Node(nodeName){
    service = this->create_service<p_01_custom_interfaces::srv::AddTwoInts>(
      "add_two_ints_service", 
      std::bind(&ServiceServer::sumTwoIntsCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

private: 
  void sumTwoIntsCallback(
    const std::shared_ptr<p_01_custom_interfaces::srv::AddTwoInts::Request> request, 
    const std::shared_ptr<p_01_custom_interfaces::srv::AddTwoInts::Response> response){
  
    response->sum = request->a + request->b;

    RCLCPP_INFO(
      get_logger(), 
      "[request] Sum : %d + %d",
      (int)request->a, 
      (int)request->b);
    
    RCLCPP_INFO(
      get_logger(),
      "[response] Result is: %d",
      (int)response->sum);
  }

  std::shared_ptr<rclcpp::Service<p_01_custom_interfaces::srv::AddTwoInts>> service;
};

int main(int argc, char **argv){

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServiceServer>("add_two_ints_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}