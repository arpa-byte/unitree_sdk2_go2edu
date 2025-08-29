#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> // For getRPY
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <memory>
#include <cmath>

// Unitree SDK
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Contact estimation gains (III-A)
static constexpr double L1 = 50.0;
static constexpr double L2 = 50.0;

class OdomBridgeNode : public rclcpp::Node {
public:
  OdomBridgeNode(const std::string &iface)
  : Node("odom_bridge_node") {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize EKF states for [x, y, yaw]
    x_ = VectorXd::Zero(3);
    P_ = MatrixXd::Identity(3, 3) * 1e-3;
    Q_ = MatrixXd::Zero(3, 3);
    Q_(0, 0) = 1e-4; Q_(1, 1) = 1e-4; Q_(2, 2) = 1e-5;
    R_ = MatrixXd::Zero(3, 3);
    R_(0, 0) = 0.5; R_(1, 1) = 0.5; R_(2, 2) = 0.2;

    last_time_ = now();

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    subscriber_->InitChannel([this](auto msg) { this->stateCallback(msg); });
  }

private:
  void stateCallback(const void *m) {
    const auto *dds = static_cast<const unitree_go::msg::dds_::SportModeState_ *>(m);
    auto now = get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0) return;

    // --- III-A Contact Estimation (placeholder) ---
    VectorXd p = VectorXd::Zero(12), pdot = VectorXd::Zero(12);
    double fz_est = L1 * (p(2) - pdot(2)) + L2 * (p(2) - pdot(2));
    bool contact = (fz_est > 1.0);

    // --- III-B Leg Odometry Twist Estimation ---
    // Use raw velocities directly, as determined in the previous step.
    double vx = dds->velocity()[0];
    double vy = dds->velocity()[1];
    double wz = dds->yaw_speed();

    if (!contact) { vx = vy = wz = 0.0; }

    // --- IV-B / III-E EKF Prediction ---
    double yaw = x_(2);
    x_(0) += (vx * cos(yaw) - vy * sin(yaw)) * dt;
    x_(1) += (vx * sin(yaw) + vy * cos(yaw)) * dt;
    x_(2) += wz * dt;
    wrapYaw(x_(2));

    MatrixXd F = MatrixXd::Identity(3, 3);
    P_ = F * P_ * F.transpose() + Q_;

    // --- EKF Correction using raw pose ---
    VectorXd z(3);
    z << dds->position()[0], dds->position()[1], 0.0;

    // Extract IMU quaternion for orientation correction
    tf2::Quaternion imu_quat(
      dds->imu_state().quaternion()[1],  // x
      dds->imu_state().quaternion()[2],  // y
      dds->imu_state().quaternion()[3],  // z
      dds->imu_state().quaternion()[0]   // w
    );

    /// ============================= FIX APPLIED HERE (replacement) =============================
    // The robot's base_link appears flipped 180° about the Y axis (forward/backwards).
    // To correct that, rotate IMU by 180° around Y and apply the same rotation to velocities.

    tf2::Quaternion rotation_fix;
    rotation_fix.setRPY(0.0, M_PI, 0.0); // 180 deg about Y (flip forward axis)

    // Apply the correction to the raw IMU quaternion
    imu_quat = rotation_fix * imu_quat;
    imu_quat.normalize();

    // IMPORTANT: rotate the body-frame velocity (vx, vy) by the same physical rotation
    // so the velocity vector and orientation stay consistent.
    tf2::Matrix3x3 Rfix(rotation_fix);
    tf2::Vector3 vel_body(vx, vy, 0.0);
    tf2::Vector3 vel_fixed = Rfix * vel_body;
    vx = vel_fixed.x();
    vy = vel_fixed.y();
    // =======================================================================================


    double roll, pitch, imu_yaw;
    tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, imu_yaw);
    z(2) = imu_yaw;

    VectorXd y = z - x_;
    wrapYaw(y(2));

    MatrixXd H = MatrixXd::Identity(3, 3);
    MatrixXd S = H * P_ * H.transpose() + R_;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    wrapYaw(x_(2));
    P_ = (MatrixXd::Identity(3, 3) - K * H) * P_;

    // --- Publish filtered odometry message ---
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = now;
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_link";
    odom->pose.pose.position.x = x_(0);
    odom->pose.pose.position.y = x_(1);

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(2));
    odom->pose.pose.orientation = tf2::toMsg(q);

    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = wz;
    odom_pub_->publish(std::move(odom));

    // --- Publish TF transform odom -> base_link ---
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_(0);
    t.transform.translation.y = x_(1);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

    last_time_ = now;
  }

  void wrapYaw(double &a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> subscriber_;

  VectorXd x_;
  MatrixXd P_, Q_, R_;
  rclcpp::Time last_time_;
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