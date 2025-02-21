#include <cmath>
#include <p_04_cpp_base_class_for_plugins/base_classes.hpp>


namespace polygon_plugins{

class Triangle : public base_classes::PolygonBase{
public:
  void initialize(double length) override {
    side_length = length;
  }

  double area() override {
    return 0.5 * side_length * getHeight();
  }

  double getHeight() {
    return sqrt((side_length * side_length) - ((side_length / 2) * (side_length / 2)));
  }

protected:
  double side_length;
}; // namespace polygon_plugins

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, base_classes::PolygonBase)