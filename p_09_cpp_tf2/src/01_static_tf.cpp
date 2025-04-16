#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class Tf2ListenerNode: public rclcpp::Node
{
public:
  Tf2ListenerNode(const std::string &node_name, std::vector<std::string> &transformation): Node(node_name){
    
    tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    this->make_transform(transformation);
  }

private:
  void make_transform(std::vector<std::string> &transformation){

    geometry_msgs::msg::TransformStamped tf_msg;

    // RCLCPP_INFO(this->get_logger(), 
    //   "\n=== Transform found === \n Translation: %s, %s, %s\n Rotation: %s, %s, %s\n",
    //   transformation[2].c_str(),
    //   transformation[3].c_str(),
    //   transformation[4].c_str(),
    //   transformation[5].c_str(),
    //   transformation[6].c_str(),
    //   transformation[7].c_str());

      tf_msg.header.stamp = this->get_clock()->now();
      tf_msg.header.frame_id = "world";

      tf_msg.child_frame_id = transformation[1];

      tf_msg.transform.translation.x = std::stod(transformation[2]);
      tf_msg.transform.translation.y = std::stod(transformation[3]);
      tf_msg.transform.translation.z = std::stod(transformation[4]);

      tf2::Quaternion q;
      q.setRPY(
        std::stod(transformation[5]),
        std::stod(transformation[6]),
        std::stod(transformation[7])
      );

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      
      tf_static_broadcaster->sendTransform(tf_msg);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  std::vector<std::string> transformation(argv, argv + argc);

  auto node = std::make_shared<Tf2ListenerNode>("tf2_listener", transformation);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
