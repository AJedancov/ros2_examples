#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Tf2PublisherNode: public rclcpp::Node
{
public:
  Tf2PublisherNode(const std::string &node_name): Node(node_name),
  x(0.0), 
  y(0.0), 
  theta(0.0),
  lin_vel(0.0), 
  ang_vel(0.0)
  {

    vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&Tf2PublisherNode::update_caommand_velocity, this, std::placeholders::_1));
    
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    using namespace std::chrono_literals;
    timer = this->create_wall_timer(
      30ms,
      std::bind(&Tf2PublisherNode::make_transform, this));

  }

private:
  void update_caommand_velocity(geometry_msgs::msg::Twist vel_msg){
    lin_vel = vel_msg.linear.x;
    ang_vel = vel_msg.angular.z;
  }

  void make_transform(){

    x += dt * lin_vel * std::cos(theta);
    y += dt * lin_vel * std::sin(theta);
    theta += dt * ang_vel;

    if (theta > M_PI) theta -= 2 * M_PI;
    if (theta < -M_PI) theta += 2 * M_PI;


    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    
    tf_broadcaster->sendTransform(tf_msg);

  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<rclcpp::TimerBase> timer;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> vel_subscription;

  double x;
  double y;
  double theta;

  double lin_vel;
  double ang_vel;

  double dt = 0.03;  // 30ms
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Tf2PublisherNode>("tf_broadcaster");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
