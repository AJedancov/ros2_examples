cmake_minimum_required(VERSION 3.8)
project(p_02_cpp_publisher_subscriber_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(p_01_custom_interfaces REQUIRED)

add_executable(publisher_address_book src/publisher_address_book.cpp)
ament_target_dependencies(publisher_address_book rclcpp p_01_custom_interfaces)

add_executable(subscriber_address_book src/subscriber_address_book.cpp)
ament_target_dependencies(subscriber_address_book rclcpp p_01_custom_interfaces)

install(TARGETS
  publisher_address_book
  subscriber_address_book
  DESTINATION lib/${PROJECT_NAME})

ament_package()