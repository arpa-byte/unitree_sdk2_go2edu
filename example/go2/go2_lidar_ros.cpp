#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>

// CORRECT INCLUDES: ChannelFactory for setup, ChannelSubscriber for data
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp" 
// CORRECT DATA TYPE: We subscribe directly to LidarState_
#include "unitree/idl/go2/LidarState_.hpp"

class LidarToROS2Node : public rclcpp::Node
{
public:
    LidarToROS2Node(const std::string& interface_name) : Node("go2_lidar_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/point_cloud", 10);
        RCLCPP_INFO(this->get_logger(), "LiDAR publisher node started, publishing on /lidar/point_cloud");

        // Set the network interface for the whole SDK
        unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

        // FINAL CORRECTED SUBSCRIBER:
        // 1. Subscribe to the LidarState_ message type.
        // 2. Listen on the "rt/lidar_state" topic.
        lidar_subscriber_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LidarState_>>(
            "rt/lidar_state");
        
        // 3. Initialize the subscriber with our callback function. This is the correct pattern.
        lidar_subscriber_->Init([this](const void* message) {
                this->LidarCallback(message);
            }, 10); // The '10' is a queue depth for the subscriber
    }

private:
    // The callback now correctly receives a LidarState_ message
    void LidarCallback(const void* message)
    {
        // Cast the message to the correct type
        auto lidar_state = static_cast<const unitree_go::msg::dds_::LidarState_*>(message);

        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        
        cloud_msg->header.stamp = this->get_clock()->now();
        cloud_msg->header.frame_id = "lidar_frame"; // Frame ID for RViz

        cloud_msg->fields.resize(4);
        cloud_msg->fields[0].name = "x";
        cloud_msg->fields[0].offset = 0;
        cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg->fields[0].count = 1;

        cloud_msg->fields[1].name = "y";
        cloud_msg->fields[1].offset = 4;
        cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg->fields[1].count = 1;

        cloud_msg->fields[2].name = "z";
        cloud_msg->fields[2].offset = 8;
        cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg->fields[2].count = 1;
        
        cloud_msg->fields[3].name = "intensity";
        cloud_msg->fields[3].offset = 12;
        cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg->fields[3].count = 1;

        // CORRECTLY ACCESS POINT CLOUD: Directly from the LidarState_ message
        const auto& lidar_points = lidar_state->point_cloud();
        size_t num_points = lidar_points.size();

        if (num_points == 0) {
            return; // Don't publish empty clouds
        }

        cloud_msg->height = 1;
        cloud_msg->width = num_points;
        cloud_msg->is_bigendian = false;
        cloud_msg->point_step = 16;
        cloud_msg->row_step = cloud_msg->point_step * num_points;
        cloud_msg->data.resize(cloud_msg->row_step);
        cloud_msg->is_dense = true;

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud_msg, "intensity");

        for (const auto& point : lidar_points)
        {
            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
            *iter_intensity = point.intensity();

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }

        publisher_->publish(std::move(cloud_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    // The subscriber member variable is now of the correct type
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LidarState_>> lidar_subscriber_;
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " network_interface" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarToROS2Node>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}