#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber(const std::string nodeName) : Node(nodeName)
  {
    subscription = this->create_subscription<std_msgs::msg::String>(
        "simpleTopic",
        10,
        std::bind(&SimpleSubscriber::topicCallback, this, std::placeholders::_1));
  }

private:
  void topicCallback(const std_msgs::msg::String &msg)
  {
    RCLCPP_INFO(get_logger(), "Recive: %s", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleSubscriber>("simple_subscriber");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
