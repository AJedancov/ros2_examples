cmake_minimum_required(VERSION 3.8)
project(p_05_cpp_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(p_04_cpp_base_class_for_plugins REQUIRED)

add_library(square_plugin SHARED
  src/plugins/square_plugin.cpp
  # src/plugins/triangle_plugin.cpp
)

add_library(triangle_plugin SHARED
  # src/plugins/square_plugin.cpp
  src/plugins/triangle_plugin.cpp
)


ament_target_dependencies(square_plugin PUBLIC pluginlib p_04_cpp_base_class_for_plugins)
ament_target_dependencies(triangle_plugin PUBLIC pluginlib p_04_cpp_base_class_for_plugins)


target_compile_features(square_plugin PUBLIC cxx_std_17)  # Require C++17
target_compile_features(triangle_plugin PUBLIC cxx_std_17)  # Require C++17

pluginlib_export_plugin_description_file(p_04_cpp_base_class_for_plugins plugins.xml)



add_executable(area_node src/area_node.cpp)

ament_target_dependencies(area_node pluginlib rclcpp p_04_cpp_base_class_for_plugins)
# target_link_libraries(area_node PUBLIC
#   pluginlib::pluginlib
#   rclcpp::rclcpp
#   p_04_cpp_base_class_for_plugins::base_classes
#   square_plugin
#   triangle_plugin
# )

install(TARGETS area_node square_plugin triangle_plugin
  DESTINATION lib/${PROJECT_NAME})

ament_package()
