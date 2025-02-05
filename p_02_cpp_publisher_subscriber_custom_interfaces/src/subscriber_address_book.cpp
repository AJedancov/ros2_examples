#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "p_01_custom_interfaces/msg/address_book.hpp"


class AddressBookSubscriber : public rclcpp::Node
{
public:
  AddressBookSubscriber(const std::string nodeName)
  : Node(nodeName)
  {
    address_book_subscriber =
      this->create_subscription<p_01_custom_interfaces::msg::AddressBook>(
        "address_book",
        10,
        std::bind(&AddressBookSubscriber::subscriberCallback, this, std::placeholders::_1));
  }

private:
  void subscriberCallback(const std::shared_ptr<p_01_custom_interfaces::msg::AddressBook> msg)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "\nReceived contact: \n" <<
      "First name: " <<
      msg->first_name.c_str() <<
      " Last name: " <<
      msg->last_name.c_str());
  }

  rclcpp::Subscription<p_01_custom_interfaces::msg::AddressBook>::SharedPtr address_book_subscriber;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<AddressBookSubscriber>("address_book_subscriber");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}