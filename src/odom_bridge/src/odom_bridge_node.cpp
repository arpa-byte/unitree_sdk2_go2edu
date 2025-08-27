#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>

#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

class OdomBridgeNode : public rclcpp::Node
{
public:
  explicit OdomBridgeNode(const std::string & interface_name)
  : rclcpp::Node("odom_bridge_node")
  {
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Odometry bridge node started with AGGRESSIVE filtering.");

    // Initialize state variables
    position_.setValue(0.0, 0.0, 0.0);
    orientation_.setRPY(0, 0, 0);
    filtered_linear_velocity_.setValue(0.0, 0.0, 0.0);
    filtered_angular_velocity_.setValue(0.0, 0.0, 0.0);
    initialized_ = false;

    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

    state_subscriber_ =
      std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    
    state_subscriber_->InitChannel([this](const void * message) { this->StateCallback(message); });
  }

private:
  void StateCallback(const void * message)
  {
    auto now = this->get_clock()->now();
    const auto * dds_msg = static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);

    if (!initialized_) {
      last_update_time_ = now;
      initialized_ = true;
      const auto& initial_quat = dds_msg->imu_state().quaternion();
      orientation_.setValue(initial_quat[1], initial_quat[2], initial_quat[3], initial_quat[0]);
      return;
    }

    double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) return;
    last_update_time_ = now;

    // --- NEW: Low-pass filter for velocities ---
    // This is the "aggressive filtering" step.
    // A smaller alpha means more smoothing (more aggressive filtering).
    const double alpha = 0.1; 
    
    // Get raw velocities
    tf2::Vector3 raw_linear_velocity(dds_msg->velocity()[0], dds_msg->velocity()[1], 0.0);
    tf2::Vector3 raw_angular_velocity(0.0, 0.0, dds_msg->yaw_speed());

    // Apply the exponential moving average filter
    filtered_linear_velocity_ = alpha * raw_linear_velocity + (1.0 - alpha) * filtered_linear_velocity_;
    filtered_angular_velocity_ = alpha * raw_angular_velocity + (1.0 - alpha) * filtered_angular_velocity_;

    // --- VELOCITY INTEGRATION (using FILTERED velocities) ---
    tf2::Vector3 linear_velocity_world = tf2::quatRotate(orientation_, filtered_linear_velocity_);
    position_ += linear_velocity_world * dt;

    tf2::Quaternion delta_rotation;
    delta_rotation.setRPY(0, 0, filtered_angular_velocity_.z() * dt);
    orientation_ = orientation_ * delta_rotation;
    orientation_.normalize();

    // --- PUBLISH THE SMOOTH, INTEGRATED ODOMETRY ---
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    odom_msg->pose.pose.position.x = position_.x();
    odom_msg->pose.pose.position.y = position_.y();
    odom_msg->pose.pose.position.z = position_.z();
    odom_msg->pose.pose.orientation = tf2::toMsg(orientation_);
    
    // Publish the FILTERED velocities in the twist message
    odom_msg->twist.twist.linear.x = filtered_linear_velocity_.x();
    odom_msg->twist.twist.linear.y = filtered_linear_velocity_.y();
    odom_msg->twist.twist.angular.z = filtered_angular_velocity_.z();
    
    odom_publisher_->publish(std::move(odom_msg));

    // Broadcast the TF transform using our integrated state
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = position_.x();
    t.transform.translation.y = position_.y();
    t.transform.translation.z = position_.z();
    t.transform.rotation = tf2::toMsg(orientation_);
    
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> state_subscriber_;

  // State variables
  bool initialized_;
  rclcpp::Time last_update_time_;
  tf2::Vector3 position_;
  tf2::Quaternion orientation_;
  tf2::Vector3 filtered_linear_velocity_;
  tf2::Vector3 filtered_angular_velocity_;
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <network_interface>\n";
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBridgeNode>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}