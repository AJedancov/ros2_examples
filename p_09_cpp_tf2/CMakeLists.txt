cmake_minimum_required(VERSION 3.8)
project(p_09_cpp_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(01_static_tf src/01_static_tf.cpp)
ament_target_dependencies(01_static_tf rclcpp tf2_ros geometry_msgs)

add_executable(02_tf_broadcaster src/02_tf_broadcaster.cpp)
ament_target_dependencies(02_tf_broadcaster rclcpp tf2_ros geometry_msgs)

add_executable(03_tf_broadcaster_vel src/03_tf_broadcaster_vel.cpp)
ament_target_dependencies(03_tf_broadcaster_vel rclcpp tf2_ros geometry_msgs)

add_executable(04_tf_broadcaster_go_to_target src/04_tf_broadcaster_go_to_target.cpp)
ament_target_dependencies(04_tf_broadcaster_go_to_target rclcpp tf2_ros tf2_geometry_msgs geometry_msgs)


install(
  TARGETS
    01_static_tf
    02_tf_broadcaster
    03_tf_broadcaster_vel
    04_tf_broadcaster_go_to_target
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
