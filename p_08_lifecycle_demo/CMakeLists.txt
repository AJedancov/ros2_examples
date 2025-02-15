cmake_minimum_required(VERSION 3.8)
project(p_08_lifecycle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# add executables
add_executable(lifecycle_publisher
  src/lifecycle_publisher.cpp)

ament_target_dependencies(lifecycle_publisher
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)


add_executable(state_manager
  src/state_manager.cpp)

ament_target_dependencies(state_manager
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)


add_executable(subscriber
  src/subscriber.cpp)

ament_target_dependencies(subscriber
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)

  
install(TARGETS
  subscriber
  lifecycle_publisher
  state_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
