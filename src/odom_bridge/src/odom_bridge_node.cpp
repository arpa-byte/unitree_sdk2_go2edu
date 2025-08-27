// odom_bridge_node.cpp
//
// Leg-Odometry-enabled odom bridge for Unitree Go2 (ROS2 Humble)
// Uses KDL to compute foot Jacobians from robot_description URDF.
// Implements leg-odometry least squares (see "Robust Localization, Mapping, and Navigation for Quadruped Robots"). :contentReference[oaicite:1]{index=1}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <vector>
#include <map>

// Unitree SDK includes (kept from your original code)
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
    this->declare_parameter<std::vector<std::string>>("foot_links",
      std::vector<std::string>({"foot_fl", "foot_fr", "foot_rl", "foot_rr"}));
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<double>("contact_vel_threshold", 0.08); // m/s
    this->declare_parameter<double>("contact_z_threshold", 0.10); // m
    this->declare_parameter<double>("damping_lambda", 1e-4);
    this->declare_parameter<double>("odom_publish_rate", 50.0);

    this->get_parameter("foot_links", foot_links_);
    this->get_parameter("base_link", base_link_);
    this->get_parameter("contact_vel_threshold", contact_vel_threshold_);
    this->get_parameter("contact_z_threshold", contact_z_threshold_);
    this->get_parameter("damping_lambda", damping_lambda_);
    double odom_rate;
    this->get_parameter("odom_publish_rate", odom_rate);
    odom_period_ = std::chrono::duration<double>(1.0 / odom_rate);

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Joint state subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&OdomBridgeNode::JointStateCallback, this, std::placeholders::_1));

    // Setup Unitree channel subscriber
    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);
    state_subscriber_ =
      std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
        "rt/sportmodestate");
    state_subscriber_->InitChannel([this](const void * message) { this->StateCallback(message); });

    // build KDL chains for each foot
    if (!setupKDL()) {
      RCLCPP_WARN(this->get_logger(), "KDL setup failed: robot_description or foot link names may be wrong. "
                   "Leg odometry will attempt fallback but full LO will be unavailable.");
      kdl_ok_ = false;
    } else {
      kdl_ok_ = true;
      RCLCPP_INFO(this->get_logger(), "KDL chains for legs created.");
    }

    // Initialize state
    ekf_x_ = VectorXd(3); ekf_x_.setZero(); // x, y, yaw
    ekf_P_ = MatrixXd::Identity(3,3) * 1e-3;
    ekf_Q_ = MatrixXd::Identity(3,3) * 1e-4;
    ekf_R_ = MatrixXd::Identity(3,3) * 1.0; // measurement trust
    last_time_ = this->get_clock()->now();

    // Make timers if desired (not necessary because data arrives from unitree), but keep heartbeat
    // (we still publish only when state callback arrives)
    RCLCPP_INFO(this->get_logger(), "Leg-odometry odometry bridge node started.");
  }

private:
  // ---------- KDL setup ----------
  bool setupKDL()
  {
    std::string robot_desc;
    if (!this->get_parameter("robot_description", robot_desc)) {
      // try param server directly
      if (!this->has_parameter("robot_description")) {
        RCLCPP_ERROR(this->get_logger(), "robot_description not on param server; KDL cannot build chains.");
        return false;
      }
      robot_desc = this->get_parameter("robot_description").as_string();
    }

    std::string urdf_xml;
    // try reading robot_description param
    try {
      rclcpp::Parameter p = this->get_parameter("robot_description");
      urdf_xml = p.as_string();
    } catch(...) {
      RCLCPP_WARN(this->get_logger(), "robot_description parameter read failed.");
      return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF to KDL tree.");
      return false;
    }

    // Build chain and solvers for each foot
    for (const auto &foot : foot_links_) {
      KDL::Chain chain;
      if (!tree.getChain(base_link_, foot, chain)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get chain from %s to %s", base_link_.c_str(), foot.c_str());
        return false;
      }
      auto jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(chain);
      auto fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
      leg_chains_.push_back(chain);
      jac_solvers_.push_back(jac_solver);
      fk_solvers_.push_back(fk_solver);
    }
    return true;
  }

  // ---------- Joint State ----------
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(js_mutex_);
    latest_js_ = *msg;
    have_js_ = true;
  }

  // Helper: map joint names to indices in a KDL chain
  bool fillJntArraysForChain(const KDL::Chain &chain,
                             const sensor_msgs::msg::JointState &js,
                             KDL::JntArray &q,
                             KDL::JntArray &qdot)
  {
    unsigned int nj = chain.getNrOfJoints();
    q = KDL::JntArray(nj);
    qdot = KDL::JntArray(nj);

    // Build a map from joint name -> index in incoming joint_state
    std::unordered_map<std::string, size_t> js_map;
    for (size_t i = 0; i < js.name.size(); ++i) js_map[js.name[i]] = i;

    // iterate chain segments and collect joint names in order
    unsigned int idx = 0;
    for (size_t s = 0; s < chain.getNrOfSegments(); ++s) {
      const KDL::Segment& seg = chain.getSegment(s);
      const KDL::Joint& joint = seg.getJoint();
      if (joint.getType() != KDL::Joint::None) {
        std::string jname = joint.getName();
        auto it = js_map.find(jname);
        if (it == js_map.end()) {
          RCLCPP_WARN(this->get_logger(), "Joint %s not found in joint_states.", jname.c_str());
          return false;
        }
        size_t js_idx = it->second;
        if (js_idx >= js.position.size()) return false;
        q(idx) = js.position[js_idx];
        if (js_idx < js.velocity.size()) qdot(idx) = js.velocity[js_idx];
        else qdot(idx) = 0.0;
        idx++;
      }
    }
    if (idx != nj) {
      RCLCPP_WARN(this->get_logger(), "Chain joints mismatch: expected %u got %u", nj, idx);
      return false;
    }
    return true;
  }

  // ---------- Leg Odometry computation ----------
  // Returns true if computed Vb (6x1) successfully
  bool computeLegOdometry(const unitree_go::msg::dds_::SportModeState_ * dds_msg,
                          VectorXd &Vb_out) // Vb_out: [vx, vy, vz, wx, wy, wz] (body frame)
  {
    // Need joint_states
    sensor_msgs::msg::JointState js_copy;
    {
      std::lock_guard<std::mutex> lock(js_mutex_);
      if (!have_js_) return false;
      js_copy = latest_js_;
    }

    // For each leg: compute foot Jacobian J (6 x n) and foot position p (KDL::Frame)
    std::vector<Eigen::Vector3d> foot_p_list;
    std::vector<Eigen::Vector3d> foot_v_list;
    std::vector<bool> contact_mask;

    for (size_t i = 0; i < leg_chains_.size(); ++i) {
      const KDL::Chain &chain = leg_chains_[i];
      KDL::JntArray q, qdot;
      if (!fillJntArraysForChain(chain, js_copy, q, qdot)) {
        // cannot compute this leg
        foot_p_list.push_back(Eigen::Vector3d::Zero());
        foot_v_list.push_back(Eigen::Vector3d::Zero());
        contact_mask.push_back(false);
        continue;
      }

      // compute FK to get foot pose
      KDL::Frame foot_frame;
      fk_solvers_[i]->JntToCart(q, foot_frame);

      // compute jacobian
      KDL::Jacobian J(chain.getNrOfJoints());
      jac_solvers_[i]->JntToJac(q, J);

      // linear velocity = Jv * qdot (first 3 rows)
      Eigen::MatrixXd J_eig(3, J.columns());
      for (size_t r = 0; r < 3; ++r) for (size_t c = 0; c < J.columns(); ++c)
        J_eig(r, c) = J(r, c);
      Eigen::VectorXd qdot_vec(J.columns());
      for (size_t k = 0; k < J.columns(); ++k) qdot_vec(k) = qdot(k);

      Eigen::Vector3d foot_v = J_eig * qdot_vec;

      // foot position in base frame (KDL uses meters)
      Eigen::Vector3d foot_p(foot_frame.p.x(), foot_frame.p.y(), foot_frame.p.z());

      // contact detection: foot linear velocity small AND foot z near ground
      bool contact = (foot_v.norm() < contact_vel_threshold_) && (foot_p.z() < contact_z_threshold_);

      foot_p_list.push_back(foot_p);
      foot_v_list.push_back(foot_v);
      contact_mask.push_back(contact);
    }

    // Build A and b for the LS problem
    // Each contact foot: Ai = [I3, -S(pi)] (3x6), bi = -vi (3x1)
    // Add IMU angular constraint: [0_3, I_3] * Vb = omega_imu

    // Count contacts
    int num_contacts = 0;
    for (bool c : contact_mask) if (c) num_contacts++;

    // Must have at least 2 contacts (2 independent feet) to have decent constraints (3*2 = 6 rows -> rank up to 6)
    if (num_contacts < 2) {
      // fallback
      return false;
    }

    int rows = num_contacts * 3 + 3; // +3 for IMU angular
    MatrixXd A = MatrixXd::Zero(rows, 6);
    VectorXd b = VectorXd::Zero(rows);

    int r = 0;
    for (size_t i = 0; i < foot_p_list.size(); ++i) {
      if (!contact_mask[i]) continue;
      Eigen::Vector3d p = foot_p_list[i];
      Eigen::Vector3d v = foot_v_list[i];

      // I3
      A.block<3,3>(r, 0) = Eigen::Matrix3d::Identity();

      // -S(p): skew-symmetric
      Eigen::Matrix3d S;
      S <<     0.0, -p.z(),  p.y(),
            p.z(),     0.0, -p.x(),
           -p.y(),  p.x(),     0.0;
      A.block<3,3>(r, 3) = -S;

      b.segment<3>(r) = -v; // bi = -vi
      r += 3;
    }

    // IMU angular velocity measurement (use dds_msg yaw/imu if available)
    // The IMU measurement is 3x; we constrain angular portion of Vb
    // AIMU = [0 0 0  I3], and b_IMU = omega_IMU
    VectorXd omega_imu = VectorXd::Zero(3);
    // Try to read IMU angular velocity from dds_msg->imu_state().gyro() or yaw_speed; adapt defensively
    // (we assume SportModeState_ has imu_state().gyro() providing 3-axis)
    try {
      auto gyro = dds_msg->imu_state().gyroscope();
      omega_imu(0) = gyro[0];
      omega_imu(1) = gyro[1];
      omega_imu(2) = gyro[2];
    } catch(...) {
      // fallback to yaw_speed on z axis only
      omega_imu(2) = dds_msg->yaw_speed();
    }

    A.block<3,3>(r, 0) = Eigen::Matrix3d::Zero();
    A.block<3,3>(r, 3) = Eigen::Matrix3d::Identity();
    b.segment<3>(r) = omega_imu;
    r += 3;

    // Solve least squares with damping (Tikhonov): (A^T A + lambda I)^{-1} A^T b
    MatrixXd AtA = A.transpose() * A;
    MatrixXd damp = damping_lambda_ * MatrixXd::Identity(6,6);
    MatrixXd inv = (AtA + damp).inverse();
    VectorXd Vb = inv * A.transpose() * b;

    Vb_out = Vb;
    return true;
  }

  // ---------- State callback (Unitree) ----------
  void StateCallback(const void * message)
  {
    const auto * dds_msg = static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) {
      last_time_ = now;
      return;
    }

    // Get Vb via leg odometry if possible
    VectorXd Vb6(6); Vb6.setZero();
    bool lo_ok = false;
    if (kdl_ok_) {
      lo_ok = computeLegOdometry(dds_msg, Vb6);
    }

    // If LO failed, fall back to Unitree velocity (as in your original code)
    double vx_body = 0.0, vy_body = 0.0, wz = 0.0;
    if (lo_ok) {
      vx_body = Vb6(0);
      vy_body = Vb6(1);
      // we only need planar angular z
      wz = Vb6(5);
    } else {
      // fallback to the provided velocities (body frame)
      vx_body = dds_msg->velocity()[0];
      vy_body = dds_msg->velocity()[1];
      wz = dds_msg->yaw_speed();
    }

    // --- EKF-like prediction using twist from leg odometry ---
    double yaw = ekf_x_(2);
    // integrate in world frame
    double dx = (vx_body * cos(yaw) - vy_body * sin(yaw)) * dt;
    double dy = (vx_body * sin(yaw) + vy_body * cos(yaw)) * dt;
    double dyaw = wz * dt;

    ekf_x_(0) += dx;
    ekf_x_(1) += dy;
    ekf_x_(2) += dyaw;

    // covariance prediction
    ekf_P_ = ekf_P_ + ekf_Q_;

    // --- Optional correction using unitree position (still helpful if position is reliable occasionally) ---
    // Use the robot provided position as a noisy measurement (like your original EKF)
    VectorXd z(3);
    z(0) = dds_msg->position()[0];
    z(1) = dds_msg->position()[1];

    // compute yaw from quaternion (defensive)
    try {
      auto q = dds_msg->imu_state().quaternion();
      double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
      double meas_yaw = atan2(2.0*(qx*qw + qy*qz), 1.0 - 2.0*(qz*qz + qw*qw));
      z(2) = meas_yaw;
    } catch(...) {
      z(2) = ekf_x_(2); // no orientation measurement
    }

    VectorXd z_pred = ekf_x_;
    VectorXd y = z - z_pred;
    // normalize yaw error
    while (y(2) > M_PI) y(2) -= 2*M_PI;
    while (y(2) < -M_PI) y(2) += 2*M_PI;

    MatrixXd H = MatrixXd::Identity(3,3);
    MatrixXd S = H * ekf_P_ * H.transpose() + ekf_R_;
    MatrixXd K = ekf_P_ * H.transpose() * S.inverse();

    ekf_x_ = ekf_x_ + K * y;
    ekf_P_ = (MatrixXd::Identity(3,3) - K * H) * ekf_P_;

    // normalize yaw
    while (ekf_x_(2) > M_PI) ekf_x_(2) -= 2*M_PI;
    while (ekf_x_(2) < -M_PI) ekf_x_(2) += 2*M_PI;

    // Publish odom
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    odom_msg->pose.pose.position.x = ekf_x_(0);
    odom_msg->pose.pose.position.y = ekf_x_(1);
    tf2::Quaternion q_out;
    q_out.setRPY(0, 0, ekf_x_(2));
    odom_msg->pose.pose.orientation = tf2::toMsg(q_out);
    odom_msg->twist.twist.linear.x = vx_body;
    odom_msg->twist.twist.linear.y = vy_body;
    odom_msg->twist.twist.angular.z = wz;
    odom_publisher_->publish(std::move(odom_msg));

    // Broadcast transform odom -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = ekf_x_(0);
    t.transform.translation.y = ekf_x_(1);
    t.transform.rotation = tf2::toMsg(q_out);
    tf_broadcaster_->sendTransform(t);

    last_time_ = now;
  }

  // ---------- members ----------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Unitree
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> state_subscriber_;

  // Joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_js_;
  std::mutex js_mutex_;
  bool have_js_{false};

  // KDL structures
  std::vector<std::string> foot_links_;
  std::string base_link_;
  std::vector<KDL::Chain> leg_chains_;
  std::vector<std::shared_ptr<KDL::ChainJntToJacSolver>> jac_solvers_;
  std::vector<std::shared_ptr<KDL::ChainFkSolverPos_recursive>> fk_solvers_;
  bool kdl_ok_{false};

  // var params
  double contact_vel_threshold_;
  double contact_z_threshold_;
  double damping_lambda_;
  std::chrono::duration<double> odom_period_;

  // EKF-like
  VectorXd ekf_x_;
  MatrixXd ekf_P_;
  MatrixXd ekf_Q_;
  MatrixXd ekf_R_;
  rclcpp::Time last_time_;

}; // class

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
