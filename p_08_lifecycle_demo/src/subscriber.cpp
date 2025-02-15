#include <memory> // for shared ptr
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class Listner : public rclcpp::Node
{
public:
  explicit Listner(const std::string &nodeName): rclcpp::Node(nodeName){

    subscription = this->create_subscription<std_msgs::msg::String>(
      "messages",
      10,
      std::bind(&Listner::subscribe_message, this, std::placeholders::_1));
  }

  void subscribe_message(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(get_logger(), "Listen message: %s", msg->data.c_str());
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscription;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Listner>("subscriber");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}