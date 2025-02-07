#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals; // to crate 3s

static constexpr char const *talker_node = "lc_talker";
static constexpr char const *node_get_state_service = "lc_talker/get_state";
static constexpr char const *node_change_state_service = "lc_talker/change_state";


class StateManager: public rclcpp::Node{
public:
  explicit StateManager(const std::string &node_name)
      : rclcpp::Node(node_name){

    // Create two clients to request two servises - get and change state
    client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_service);
    client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_service);

    RCLCPP_INFO(get_logger(), "State Manager created");
  }


  unsigned int get_state(std::chrono::seconds timeout = 3s){

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state->wait_for_service(timeout)){
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not avaible",
        client_get_state->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = client_get_state->async_send_request(request);
    // auto future_status = wait_for_result(future_result, timeout);
    auto future_status = future_result.wait_for(std::chrono::milliseconds(100));


    if (future_status != std::future_status::ready){
      RCLCPP_ERROR(
        get_logger(),
        "Service timed out while getting current state of node %s",
        talker_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if(future_result.get()){
      auto state = future_result.get()->current_state.id;

      RCLCPP_INFO(
        get_logger(),
        "Node %s has current state %s",
        talker_node,
        future_result.get()->current_state.label.c_str());

      return state;
    }else{
      RCLCPP_ERROR(
        get_logger(),
        "Failed to get the current state of node %s",
        talker_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }


  bool change_state(std::uint8_t transition){

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = transition;

    if (!client_change_state->wait_for_service(std::chrono::milliseconds(3000))){
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not avaible",
        client_get_state->get_service_name());
      return false;
    }

    auto future_result = client_change_state->async_send_request(request);
    // auto future_status = wait_for_result(future_result, timeout);
    auto future_status = future_result.wait_for(std::chrono::milliseconds(100));

    if (future_status != std::future_status::ready){
      RCLCPP_ERROR(
        get_logger(),
        "Service timed out while changig current state of node %s",
        talker_node);
      return false;
    }

    if (future_result.get()->success){
      RCLCPP_INFO(
        get_logger(),
        "Transition %d succesfully triggered",
        static_cast<unsigned int>(transition));
      return true;
    }else{
      RCLCPP_WARN(
        get_logger(),
        "Failed to triger transition %d for node %s",
        static_cast<unsigned int>(transition),
        talker_node);
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state;
};

// To interact with Service Client and chage its states we can use simple script
void state_manager(std::shared_ptr<StateManager> node, const uint timeout_between_states){

  rclcpp::WallRate state_change_dalay(timeout_between_states); 

  std::vector<std::uint8_t> states = {
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
    lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN
  };

  for(auto state : states){
    if(!node->change_state(state) || !node->get_state()){
      return;
    }
    state_change_dalay.sleep();
  }
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StateManager>("state_manager");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); 

  std::shared_future<void> state_manager_future = std::async(
    std::launch::async,
    std::bind(state_manager, node, 0.1)); //10s 

  executor.spin_until_future_complete(state_manager_future);

  rclcpp::shutdown();

  return 0;
}