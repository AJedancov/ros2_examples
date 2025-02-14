#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals; // to crate 3s

static constexpr char const *talker_node = "lc_talker";
static constexpr char const *get_state_service = "lc_talker/get_state";
static constexpr char const *change_state_service = "lc_talker/change_state";


class StateManager: public rclcpp::Node{
public:
  explicit StateManager(const std::string &node_name): rclcpp::Node(node_name){
    // timer = this->create_wall_timer(0s, std::bind(&StateManager::swich_states, this));
    swich_states();
  }


private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state;
  std::shared_ptr<rclcpp::TimerBase> timer;


  void swich_states(){
    // timer->cancel(); // Run this function only once

    RCLCPP_INFO(get_logger(), "State Manager created");

    // Create two clients to request two servises - get and change state
    client_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_service);
    client_get_state = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service);

    std::vector<std::uint8_t> states = {
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
      lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN
    };


    rclcpp::WallRate state_change_dalay(3s);

    for(auto state : states){
      if(!change_state(state, 3s) || !get_state(3s)){
        RCLCPP_ERROR(get_logger(), "Unable to change state");
        return;
      }
      state_change_dalay.sleep();
    }
    RCLCPP_INFO(get_logger(), "All states successfully changed");
  }


  bool change_state(std::uint8_t transition, std::chrono::seconds timeout = 5s){

    if(!client_change_state->wait_for_service(timeout)){
      RCLCPP_ERROR(get_logger(),
        "[change_state]: Service %s is not avaible",
        client_change_state->get_service_name());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    auto future = client_change_state->async_send_request(request);

    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS){

      RCLCPP_INFO(get_logger(),
        "[change_state]: Transition %d succesfully triggered",
        static_cast<unsigned int>(transition));
      return true;
    }else{
      RCLCPP_WARN(get_logger(),
        "[change_state]: Failed to triger transition %d for node %s",
        static_cast<unsigned int>(transition),
        talker_node);
      return false;
    }
  }


  unsigned int get_state(std::chrono::seconds timeout = 5s){
    
    if(!client_get_state->wait_for_service(timeout)){
      RCLCPP_ERROR(get_logger(), "[get_state]: Service %s is not avaible",
      client_get_state->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client_get_state->async_send_request(request);

    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS){
      
      auto state = future.get();

      RCLCPP_INFO(get_logger(), "[get_state]: Node %s has current state %s",
        talker_node,
        state->current_state.label.c_str());
      return state->current_state.id;
    }else{
      RCLCPP_ERROR(get_logger(),"[get_state]: Failed to get the current state of node %s",
        talker_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StateManager>("state_manager");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); 
  executor.spin();

  rclcpp::shutdown();

  return 0;
}