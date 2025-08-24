#include <rclcpp/rclcpp.hpp>
#include <iomanip>
#include <iostream>

// Unitree SDK Includes
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"

class OdomTesterNode : public rclcpp::Node
{
public:
    OdomTesterNode(const std::string& interface_name) : Node("odom_tester_node")
    {
        RCLCPP_INFO(this->get_logger(), "Odometry tester node started.");

        unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

        sport_mode_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/sportmodestate");
        
        sport_mode_subscriber_->InitChannel([this](const void* message) {
            this->SportModeCallback(message);
        });
    }

private:
    void SportModeCallback(const void* message)
    {
        auto state = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

        // --- CORRECTED DATA ACCESS ---
        const auto& position = state->position();
        // The quaternion is inside the imu_state() object
        const auto& orientation_quat = state->imu_state().quaternion();
        const auto& linear_velocity = state->velocity();
        // The angular velocity (gyroscope data) is also inside the imu_state() object
        const auto& angular_velocity = state->imu_state().gyroscope();

        RCLCPP_INFO(this->get_logger(), "--- ODOMETRY DATA RECEIVED ---");
        std::cout << std::fixed << std::setprecision(4)
                  // The quaternion array order is [w, x, y, z]
                  << "Position (x, y, z):    [" << position[0] << ", " << position[1] << ", " << position[2] << "]" << std::endl
                  << "Orientation (x, y, z, w): [" << orientation_quat[1] << ", " << orientation_quat[2] << ", " << orientation_quat[3] << ", " << orientation_quat[0] << "]" << std::endl
                  << "Linear Velocity (x, y, z):  [" << linear_velocity[0] << ", " << linear_velocity[1] << ", " << linear_velocity[2] << "]" << std::endl
                  << "Angular Velocity (x, y, z): [" << angular_velocity[0] << ", " << angular_velocity[1] << ", " << angular_velocity[2] << "]" << std::endl
                  << std::endl;
    }

    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport_mode_subscriber_;
};

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " network_interface" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTesterNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}