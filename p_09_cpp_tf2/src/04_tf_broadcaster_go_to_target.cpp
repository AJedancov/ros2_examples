#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

class Tf2PublisherNode: public rclcpp::Node
{
public:
  Tf2PublisherNode(const std::string &node_name): Node(node_name),
  x(0.0), 
  y(0.0), 
  theta(0.0),
  x_target(-2.0),
  y_target(2.0),
  theta_target(M_PI/4),
  lin_vel(0.0), 
  ang_vel(0.0),
  dt(0.03) // 30ms
  {

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        

    tf2::Quaternion q;

    target_pose.position.x = x_target;
    target_pose.position.y = y_target;
    q.setRPY(0, 0, theta_target);
    target_pose.orientation = tf2::toMsg(q);
    this->update_frame(world_frame_name, target_frame_name, target_pose, false);

    pose.position.x = x;
    pose.position.y = y;
    q.setRPY(0, 0, theta);
    pose.orientation = tf2::toMsg(q);
    this->update_frame(world_frame_name, base_link_frame_name, pose, false);

    timer = this->create_wall_timer(
      0.03s,
      std::bind(&Tf2PublisherNode::update, this)
    );
  }

private:

  void update_frame(
    const std::string &paretn_frame,
    const std::string &child_frame,
    geometry_msgs::msg::Pose &pose, 
    bool is_dynamic)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = paretn_frame;
    tf_msg.child_frame_id = child_frame;

    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = 0.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, pose.orientation.z);
    tf_msg.transform.rotation.x = pose.orientation.x;
    tf_msg.transform.rotation.y = pose.orientation.y;
    tf_msg.transform.rotation.z = pose.orientation.z;
    tf_msg.transform.rotation.w = pose.orientation.w;
    
    // RCLCPP_INFO(get_logger(), "\n  create parent frame: %s\n  and child frame: %s", paretn_frame.c_str(), child_frame.c_str());
  
    if(is_dynamic)
      tf_broadcaster->sendTransform(tf_msg);
    else
      tf_static_broadcaster->sendTransform(tf_msg);
  }

  void update(){

    // geometry_msgs::msg::TransformStamped tf_lockup;
    // tf_lockup = tf_buffer->lookupTransform(
    //   target_frame_name, base_link_frame_name,
    //   tf2::TimePointZero
    // );


    distance_error = std::hypot(
      target_pose.position.x - pose.position.x, 
      target_pose.position.y - pose.position.y
    );

    angle_to_target = std::atan2(
      target_pose.position.y - pose.position.y,
      target_pose.position.x - pose.position.x
    );

    angle_error = angle_to_target - theta;

    // Angle normalization
    if(angle_error > M_PI) angle_error -= 2 * M_PI;
    if(angle_error < -M_PI) angle_error += 2 * M_PI;
    
    
    lin_vel = 0.5 * distance_error;
    ang_vel = 2.5 * angle_error;

    RCLCPP_INFO(get_logger(), "\n  lin_vel: %lf\n  ang_vel: %lf", lin_vel, ang_vel);
    
    x += dt * lin_vel * std::cos(theta);
    y += dt * lin_vel * std::sin(theta);
    theta += dt * ang_vel;

    pose.position.x = x;
    pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.orientation = tf2::toMsg(q);
    this->update_frame(world_frame_name, base_link_frame_name, pose, true);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;

  std::shared_ptr<rclcpp::TimerBase> timer;

  double x;
  double y;
  double theta;
  
  double x_target;
  double y_target;
  double theta_target;

  geometry_msgs::msg::Pose target_pose;
  geometry_msgs::msg::Pose pose;

  std::string world_frame_name = "world";
  std::string base_link_frame_name = "base_link";
  std::string target_frame_name = "target";


  double lin_vel;
  double ang_vel;

  double distance_error;
  double angle_to_target;
  double angle_error;

  double dt;  
};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Tf2PublisherNode>("tf_broadcaster_target");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
