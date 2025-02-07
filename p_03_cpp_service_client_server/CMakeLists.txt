cmake_minimum_required(VERSION 3.8)
project(p_03_cpp_service_client_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(p_01_custom_interfaces REQUIRED)


add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp p_01_custom_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp p_01_custom_interfaces)

install(
  TARGETS
    add_two_ints_server
    add_two_ints_client
  DESTINATION lib/${PROJECT_NAME})



ament_package()
