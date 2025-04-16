#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Tf2PublisherNode: public rclcpp::Node
{
public:
  Tf2PublisherNode(const std::string &node_name): Node(node_name){
    
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    using namespace std::chrono_literals;
    timer = this->create_wall_timer(
      1s,
      std::bind(&Tf2PublisherNode::make_transform, this));
  }

private:
  void make_transform(){

    geometry_msgs::msg::TransformStamped tf_msg;

      tf_msg.header.stamp = this->get_clock()->now();
      tf_msg.header.frame_id = "world";

      tf_msg.child_frame_id = "base_link";

      tf_msg.transform.translation.x = pos_x;
      tf_msg.transform.translation.y = 0;
      tf_msg.transform.translation.z = 0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      
      tf_broadcaster->sendTransform(tf_msg);

      pos_x += 0.1; // x_k1 = x_k0 + 0.1
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<rclcpp::TimerBase> timer;
  double pos_x = 0.0;
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Tf2PublisherNode>("tf_broadcaster");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
