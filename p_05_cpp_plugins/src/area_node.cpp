#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <p_04_cpp_base_class_for_plugins/base_classes.hpp>

class CallerPlugins : public rclcpp::Node{
public:
  CallerPlugins(const std::string node_name) : Node(node_name){
    
    // console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

    pluginlib::ClassLoader<base_classes::PolygonBase> poly_loader("p_04_cpp_base_class_for_plugins", "base_classes::PolygonBase");
        
    try{
      square = poly_loader.createSharedInstance("polygon_plugins::Square");
      square->initialize(10.0);

      triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
      triangle->initialize(10.0);
      
      // square = std::move(poly_loader.createUniqueInstance("polygon_plugins::Square"));
      // square->initialize(10.0);

      // triangle = std::move(poly_loader.createUniqueInstance("polygon_plugins::Triangle"));
      // triangle->initialize(10.0);

      RCLCPP_INFO(get_logger(), "Square area: %lf\n", square->area());  
      RCLCPP_INFO(get_logger(), "Triangle area: %lf\n", triangle->area());
    }
    catch(pluginlib::PluginlibException& ex){
      printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }
    triangle.reset();
    square.reset();
  }


private:
  std::shared_ptr<base_classes::PolygonBase> triangle;
  std::shared_ptr<base_classes::PolygonBase> square;
  // pluginlib::UniquePtr<base_classes::PolygonBase> triangle;
  // pluginlib::UniquePtr<base_classes::PolygonBase> square;

};


int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CallerPlugins>("area_node");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}