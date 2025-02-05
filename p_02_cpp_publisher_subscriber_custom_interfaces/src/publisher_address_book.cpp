#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "p_01_custom_interfaces/msg/address_book.hpp"

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher(const std::string nodeName)
  : Node(nodeName)
  {
    address_book_publisher =
      this->create_publisher<p_01_custom_interfaces::msg::AddressBook>("address_book", 10);

    using namespace std::chrono_literals;
    timer = this->create_wall_timer(
        1s,
        bind(&AddressBookPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    auto msg = p_01_custom_interfaces::msg::AddressBook();

    msg.first_name = "John";
    msg.last_name = "J.";
    msg.phone_number = "1234567890";
    msg.phone_type = msg.PHONE_TYPE_MOBILE;

    RCLCPP_INFO_STREAM(
      get_logger(),
      "\nPublishing contact:\n" <<
      "First name: " <<
       msg.first_name.c_str() <<
       " Last name:  " <<
       msg.last_name.c_str());

    this->address_book_publisher->publish(msg);
  }

  rclcpp::Publisher<p_01_custom_interfaces::msg::AddressBook>::SharedPtr address_book_publisher;
  rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<AddressBookPublisher>("address_book_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}