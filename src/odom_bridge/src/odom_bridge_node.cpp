#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> // Required for getRPY
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

// Contact estimation gains (from III-A)
static constexpr double L1 = 50.0;
static constexpr double L2 = 50.0;

class OdomBridgeNode : public rclcpp::Node {
public:
  OdomBridgeNode(const std::string &iface)
  : Node("odom_bridge_node") {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // EKF state: [x, y, yaw]
    x_    = VectorXd::Zero(3);
    P_    = MatrixXd::Identity(3,3) * 1e-3;
    Q_    = MatrixXd::Zero(3,3);
    Q_(0,0)=1e-4; Q_(1,1)=1e-4; Q_(2,2)=1e-5;
    R_    = MatrixXd::Zero(3,3);
    R_(0,0)=0.5;  R_(1,1)=0.5;  R_(2,2)=0.2;

    last_time_ = now();

    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    subscriber_->InitChannel([this](auto msg){ this->stateCallback(msg); });
  }

private:
  void stateCallback(const void *m) {
    const auto *dds = static_cast<const unitree_go::msg::dds_::SportModeState_*>(m);
    auto now = get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0) return;

    // --- III-A Contact Estimation ---
    VectorXd p = VectorXd::Zero(12), pdot = VectorXd::Zero(12);
    double fz_est = L1*(p(2) - pdot(2)) + L2*(p(2) - pdot(2));
    bool contact = (fz_est > 1.0);

    // --- III-B Leg Odometry Twist Estimation ---
    double raw_vx = dds->velocity()[0];
    double raw_vy = dds->velocity()[1];
    double raw_wz = dds->yaw_speed();

    // ********************************************************************
    // *** BEGIN FIX: Flip coordinate system to match ROS REP-103         ***
    // *** We negate all body-frame velocities because the robot's native ***
    // *** coordinate system is flipped 180 degrees.                    ***
    // ********************************************************************
    double vx = -raw_vx;
    double vy = -raw_vy; // Also flip Y to maintain a right-hand coordinate system
    double wz = -raw_wz;
    // ********************************************************************
    // *** END FIX                                                      ***
    // ********************************************************************

    if (!contact) { vx = vy = wz = 0.0; }

    // --- IV-B / III-E EKF Prediction ---
    // Use the corrected velocities to predict the next state in the correct frame.
    double yaw = x_(2);
    x_(0) += (vx * cos(yaw) - vy * sin(yaw)) * dt;
    x_(1) += (vx * sin(yaw) + vy * cos(yaw)) * dt;
    x_(2) += wz * dt;
    wrapYaw(x_(2));

    MatrixXd F = MatrixXd::Identity(3,3);
    P_ = F * P_ * F.transpose() + Q_;

    // --- EKF Correction using raw pose ---
    VectorXd z(3);
    z << dds->position()[0], dds->position()[1], 0;

    // ********************************************************************
    // *** BEGIN FIX: Correct the IMU orientation (180-degree yaw rotation) ***
    // ********************************************************************
    // --- FIXED: Use IMU quaternion directly, no extra 180° yaw flip ---
    tf2::Quaternion imu_quat(
        dds->imu_state().quaternion()[1], // x
        dds->imu_state().quaternion()[2], // y
        dds->imu_state().quaternion()[3], // z
        dds->imu_state().quaternion()[0]  // w
    );
    imu_quat.normalize();

    double corrected_roll, corrected_pitch, corrected_yaw;
    tf2::Matrix3x3(imu_quat).getRPY(corrected_roll, corrected_pitch, corrected_yaw);
    z(2) = corrected_yaw; // Use the corrected yaw directly                                   
    // ********************************************************************
    // *** END FIX                                                      ***
    // ********************************************************************

    VectorXd y = z - x_;
    wrapYaw(y(2));

    MatrixXd H = MatrixXd::Identity(3,3);
    MatrixXd S = H*P_*H.transpose() + R_;
    MatrixXd K = P_*H.transpose()*S.inverse();
    x_ = x_ + K*y;
    wrapYaw(x_(2));
    P_ = (MatrixXd::Identity(3,3) - K*H) * P_;

    // --- PUBLISH filtered odometry ---
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = now;
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_link";
    odom->pose.pose.position.x = x_(0);
    odom->pose.pose.position.y = x_(1);

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(2));
    odom->pose.pose.orientation = tf2::toMsg(q);

    // Publish the corrected twist for downstream nodes
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = wz;
    odom_pub_->publish(*odom);

    // --- TF broadcast ---
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
    while(a > M_PI)  a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> subscriber_;

  VectorXd x_;
  MatrixXd P_, Q_, R_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomBridgeNode>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}