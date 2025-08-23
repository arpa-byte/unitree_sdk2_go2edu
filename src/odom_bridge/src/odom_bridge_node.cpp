#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Unitree SDK Includes
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

class OdomBridgeNode : public rclcpp::Node
{
public:
  explicit OdomBridgeNode(const std::string & interface_name)
  : rclcpp::Node("odom_bridge_node")
  {
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Odometry bridge node started.");

    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

    state_subscriber_ =
      std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    
    state_subscriber_->InitChannel([this](const void * message) { this->StateCallback(message); });
  }

private:
  void StateCallback(const void * message)
  {
    const auto * dds_msg = static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    auto now = this->get_clock()->now();

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    // Position (Correct)
    odom_msg->pose.pose.position.x = dds_msg->position()[0];
    odom_msg->pose.pose.position.y = dds_msg->position()[1];
    odom_msg->pose.pose.position.z = dds_msg->position()[2];

    // --- CRITICAL FIX: Access the quaternion through imu_state() ---
    odom_msg->pose.pose.orientation.w = dds_msg->imu_state().quaternion()[0];
    odom_msg->pose.pose.orientation.x = dds_msg->imu_state().quaternion()[1];
    odom_msg->pose.pose.orientation.y = dds_msg->imu_state().quaternion()[2];
    odom_msg->pose.pose.orientation.z = dds_msg->imu_state().quaternion()[3];

    // Velocity (Correct)
    odom_msg->twist.twist.linear.x = dds_msg->velocity()[0];
    odom_msg->twist.twist.linear.y = dds_msg->velocity()[1];
    odom_msg->twist.twist.linear.z = dds_msg->velocity()[2];
    odom_msg->twist.twist.angular.z = dds_msg->yaw_speed();
    
    odom_publisher_->publish(std::move(odom_msg));

    // Broadcast the TF transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = dds_msg->position()[0];
    t.transform.translation.y = dds_msg->position()[1];
    t.transform.translation.z = dds_msg->position()[2];

    // --- CRITICAL FIX: Use the same correct quaternion for the transform ---
    t.transform.rotation.w = dds_msg->imu_state().quaternion()[0];
    t.transform.rotation.x = dds_msg->imu_state().quaternion()[1];
    t.transform.rotation.y = dds_msg->imu_state().quaternion()[2];
    t.transform.rotation.z = dds_msg->imu_state().quaternion()[3];
    
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>
    state_subscriber_;
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