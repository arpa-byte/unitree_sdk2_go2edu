#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <Eigen/Dense> // For matrix math

// Unitree SDK Includes
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class OdomBridgeNode : public rclcpp::Node
{
public:
  explicit OdomBridgeNode(const std::string & interface_name)
  : rclcpp::Node("odom_bridge_node")
  {
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Odometry bridge node with EKF filter started.");

    // Initialize EKF state and covariance
    // State: [x, y, yaw]
    ekf_x_ = VectorXd(3);
    ekf_x_.setZero();
    ekf_P_ = MatrixXd::Identity(3, 3) * 1e-3;

    // Process Noise Covariance (Q): Trust prediction almost entirely
    ekf_Q_ = MatrixXd::Zero(3, 3);
    ekf_Q_(0, 0) = 1e-8;  // almost no process noise in position
    ekf_Q_(1, 1) = 1e-8;
    ekf_Q_(2, 2) = 1e-6;  // very low process noise in yaw

    // Measurement Noise Covariance (R): Ignore jumpy measurements almost completely
    ekf_R_ = MatrixXd::Zero(3, 3);
    ekf_R_(0, 0) = 1e3;   // very high measurement noise for x
    ekf_R_(1, 1) = 1e3;   // very high measurement noise for y
    ekf_R_(2, 2) = 100.0; // very high measurement noise for yaw

    last_time_ = this->get_clock()->now();

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
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    // --- EKF PREDICTION STEP ---
    double yaw = ekf_x_(2);
    double vx = dds_msg->velocity()[0];
    double vy = dds_msg->velocity()[1];
    double yaw_rate = dds_msg->yaw_speed();

    // Predict state using velocity (control input)
    ekf_x_(0) += (vx * cos(yaw) - vy * sin(yaw)) * dt;
    ekf_x_(1) += (vx * sin(yaw) + vy * cos(yaw)) * dt;
    ekf_x_(2) += yaw_rate * dt;

    // Predict covariance
    MatrixXd F = MatrixXd::Identity(3,3);
    ekf_P_ = F * ekf_P_ * F.transpose() + ekf_Q_;

    // --- EKF CORRECTION STEP ---
    // Measurement vector: [x_raw, y_raw, yaw_raw]
    VectorXd z(3);
    z(0) = dds_msg->position()[0];
    z(1) = dds_msg->position()[1];
    
    auto q = dds_msg->imu_state().quaternion();
    z(2) = atan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]));
    
    VectorXd z_pred = ekf_x_; // Measurement model is direct identity
    VectorXd y = z - z_pred; // Innovation (error)

    // Normalize yaw error
    while (y(2) > M_PI) y(2) -= 2*M_PI;
    while (y(2) < -M_PI) y(2) += 2*M_PI;

    MatrixXd H = MatrixXd::Identity(3,3);
    MatrixXd S = H * ekf_P_ * H.transpose() + ekf_R_;
    MatrixXd K = ekf_P_ * H.transpose() * S.inverse();

    // Update state and covariance with the correction
    ekf_x_ = ekf_x_ + K * y;
    ekf_P_ = (MatrixXd::Identity(3,3) - K * H) * ekf_P_;

    // Normalize final yaw
    while (ekf_x_(2) > M_PI) ekf_x_(2) -= 2*M_PI;
    while (ekf_x_(2) < -M_PI) ekf_x_(2) += 2*M_PI;

    // --- PUBLISH FILTERED ODOMETRY ---
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    odom_msg->pose.pose.position.x = ekf_x_(0);
    odom_msg->pose.pose.position.y = ekf_x_(1);
    
    tf2::Quaternion q_corrected;
    q_corrected.setRPY(0, 0, ekf_x_(2));
    odom_msg->pose.pose.orientation = tf2::toMsg(q_corrected);

    odom_msg->twist.twist.linear.x = vx;
    odom_msg->twist.twist.linear.y = vy;
    odom_msg->twist.twist.angular.z = yaw_rate;
    odom_publisher_->publish(std::move(odom_msg));

    // Broadcast filtered TF transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = ekf_x_(0);
    t.transform.translation.y = ekf_x_(1);
    t.transform.rotation = tf2::toMsg(q_corrected);
    tf_broadcaster_->sendTransform(t);

    last_time_ = now;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> state_subscriber_;

  // EKF variables
  VectorXd ekf_x_;
  MatrixXd ekf_P_;
  MatrixXd ekf_Q_;
  MatrixXd ekf_R_;
  rclcpp::Time last_time_;
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