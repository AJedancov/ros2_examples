#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "p_01_custom_interfaces/srv/add_two_ints.hpp"


class ServiceClient: public rclcpp::Node{
public:
  ServiceClient(const std::string nodeName)
  : rclcpp::Node(nodeName), a(2), b(3)
  {

    client = this->create_client<p_01_custom_interfaces::srv::AddTwoInts>("add_two_ints_service");

    while (!client->wait_for_service(std::chrono::milliseconds(1000))){
      RCLCPP_INFO(get_logger(), "Service server not available...");
    }

    auto request = std::make_shared<p_01_custom_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    RCLCPP_INFO(get_logger(), "[request] Sum: %d + %d", a, b);

    auto future = client->async_send_request(request);

    // Wait for the result from Service Server
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS){

      auto response = future.get();
      RCLCPP_INFO(get_logger(), "[response] Result is: %d", (int)response->sum);
    }else{
      RCLCPP_ERROR(get_logger(), "Service call failed");
    }
  }

private:
  std::shared_ptr<rclcpp::Client<p_01_custom_interfaces::srv::AddTwoInts>> client;
  int a, b;
};

int main(int argc, char **argv){

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServiceClient>("add_two_ints_client");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
