#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string &nodeName)
  : LifecycleNode(nodeName){

    RCLCPP_INFO(get_logger(), "Lifecycle Talker created");
  }


  // === Lifecycle Node specific methods ===
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State &){

    publisher = this->create_publisher<std_msgs::msg::String>("messages", 10);

    using namespace std::chrono_literals; // to crate 1s
    timer = this->create_wall_timer(
        500ms,
        std::bind(&LifecycleTalker::publish_message, this));

    RCLCPP_INFO(get_logger(), "on_configure() called");
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State &){
    publisher->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate() called");
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &){
    publisher->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate() called");
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &){
    publisher.reset();
    timer.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() called");
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(get_logger(), "on_shutdown() called");
    return CallbackReturnT::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher;
  std::shared_ptr<rclcpp::TimerBase> timer;

  void publish_message(){
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Hello";

    if (!publisher->is_activated()){
      RCLCPP_WARN(get_logger(), "Publisher inactive");
    }else{
      RCLCPP_INFO(get_logger(), "Publisch message: %s", msg->data.c_str());
      publisher->publish(std::move(msg));
    }
  }
};

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LifecycleTalker>("lc_publisher");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}