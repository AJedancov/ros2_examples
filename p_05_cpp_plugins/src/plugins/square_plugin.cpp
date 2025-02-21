#include <p_04_cpp_base_class_for_plugins/base_classes.hpp>

namespace polygon_plugins{
 
class Square : public base_classes::PolygonBase{
public:
  void initialize(double side_length) override {
    this->side_length = side_length;
  }

  double area() override {
    return side_length * side_length;
  }

protected:
  double side_length;
};

} // namespace polygon_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, base_classes::PolygonBase)