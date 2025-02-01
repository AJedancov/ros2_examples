#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher(const std::string nodeName) : Node(nodeName)
  {
    publisher = this->create_publisher<std_msgs::msg::String>("simpleTopic", 10);

    using namespace std::chrono_literals; // for time declaration in s

    timer = this->create_wall_timer(
      1s,
      std::bind(&SimplePublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Important data";
    RCLCPP_INFO(get_logger(), "Publish: %s", msg->data.c_str());
    publisher->publish(std::move(msg));
  }

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher;
  std::shared_ptr<rclcpp::TimerBase> timer;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimplePublisher>("simple_publisher");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
