#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp> // New include for IMU messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <cmath>

// Unitree SDK
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

class OdomBridgeNode : public rclcpp::Node {
public:
  OdomBridgeNode(const std::string &iface) : Node("odom_bridge_node") {
    // This parameter controls whether this node publishes the TF transform.
    // We will set this to false in the launch file, as robot_localization will do it.
    this->declare_parameter("publish_tf", true);
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50); // New IMU publisher
    
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    last_time_ = now();

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    subscriber_->InitChannel([this](auto msg) { this->stateCallback(msg); });

    RCLCPP_INFO(this->get_logger(), "Odom bridge started. Publishing raw odom and IMU data.");
  }

private:
  void stateCallback(const void *m) {
    const auto *dds = static_cast<const unitree_go::msg::dds_::SportModeState_ *>(m);
    auto now = get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0) return;

    double raw_vx = dds->velocity()[0];
    double raw_vy = dds->velocity()[1];
    double raw_wz = dds->yaw_speed();

    // --- Publish IMU Data ---
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = now;
    imu_msg->header.frame_id = "base_link"; // IMU is relative to the robot's base
    imu_msg->orientation.w = dds->imu_state().quaternion()[0];
    imu_msg->orientation.x = dds->imu_state().quaternion()[1];
    imu_msg->orientation.y = dds->imu_state().quaternion()[2];
    imu_msg->orientation.z = dds->imu_state().quaternion()[3];
    imu_msg->angular_velocity.x = dds->imu_state().gyroscope()[0];
    imu_msg->angular_velocity.y = dds->imu_state().gyroscope()[1];
    imu_msg->angular_velocity.z = dds->imu_state().gyroscope()[2];
    imu_msg->linear_acceleration.x = dds->imu_state().accelerometer()[0];
    imu_msg->linear_acceleration.y = dds->imu_state().accelerometer()[1];
    imu_msg->linear_acceleration.z = dds->imu_state().accelerometer()[2];
    imu_pub_->publish(std::move(imu_msg));

    // --- Dead-Reckoning Integration ---
    double vx = raw_vx;
    double vy = raw_vy;
    double wz = raw_wz;

    double current_yaw = yaw_;
    x_ += (vx * cos(current_yaw) - vy * sin(current_yaw)) * dt;
    y_ += (vx * sin(current_yaw) + vy * cos(current_yaw)) * dt;
    yaw_ += wz * dt;
    wrapYaw(yaw_);

    // --- Publish Odometry Message (NO TF) ---
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = now;
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_link";
    odom->pose.pose.position.x = x_;
    odom->pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom->pose.pose.orientation = tf2::toMsg(q);
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = wz;
    odom_pub_->publish(std::move(odom));

    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
    }

    last_time_ = now;
  }

  void wrapYaw(double &a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // IMU publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // --- THIS LINE WAS MISSING ---
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> subscriber_;

  double x_, y_, yaw_;
  rclcpp::Time last_time_;
  bool publish_tf_;
};

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <network_interface>\n";
    return 1;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBridgeNode>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}