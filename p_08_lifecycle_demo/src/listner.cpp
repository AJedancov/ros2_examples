#include <memory> // for shared ptr
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

class Listner : public rclcpp::Node
{
public:
  explicit Listner(const std::string &nodeName)
  : rclcpp::Node(nodeName)
  {
    messageSubscription = this->create_subscription<std_msgs::msg::String>(
      "messages",
      10,
      std::bind(&Listner::messageCallback, this, std::placeholders::_1));

    // Create Subscription to check evnts changes
    notificationSubscription = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "lc_talker/transition_event",
      10,
      std::bind(&Listner::notificationCallback, this, std::placeholders::_1));
  }

  void messageCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "messageCallback: %s", msg->data.c_str());
  }

  void notificationCallback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "notificationCallback: transition from %s to %s",
      msg->start_state.label.c_str(),
      msg->goal_state.label.c_str());
  }

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> messageSubscription;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> notificationSubscription;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  std::shared_ptr listnerNode = std::make_shared<Listner>("listner_node");
  rclcpp::spin(listnerNode);

  rclcpp::shutdown();
  return 0;
}