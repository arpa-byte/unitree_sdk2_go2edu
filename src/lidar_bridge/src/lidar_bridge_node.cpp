#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/ros2/PointCloud2_.hpp"

class LidarToROS2Node : public rclcpp::Node
{
public:
  explicit LidarToROS2Node(const std::string & interface_name)
  : rclcpp::Node("lidar_bridge_node")
  {
    // Publish on /lidar/point_cloud with reasonable depth
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/point_cloud", qos);
    RCLCPP_INFO(this->get_logger(), "LiDAR publisher node started.");

    unitree::robot::ChannelFactory::Instance()->Init(0, interface_name);

    lidar_subscriber_ =
      std::make_shared<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(
        "rt/utlidar/cloud");

    lidar_subscriber_->InitChannel([this](const void * message) { this->LidarCallback(message); });
  }

private:
  void LidarCallback(const void * message)
  {
    const auto * dds_msg = static_cast<const sensor_msgs::msg::dds_::PointCloud2_ *>(message);
    auto ros2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // --- CRITICAL: stamp with current ROS time (system time on Humble) ---
    ros2_msg->header.stamp = this->get_clock()->now();

    // Use a consistent frame name that matches your static TF
    ros2_msg->header.frame_id = "utlidar_lidar";

    // Copy fields
    ros2_msg->height = dds_msg->height();
    ros2_msg->width = dds_msg->width();

    ros2_msg->fields.resize(dds_msg->fields().size());
    for (size_t i = 0; i < dds_msg->fields().size(); ++i) {
      ros2_msg->fields[i].name = dds_msg->fields()[i].name();
      ros2_msg->fields[i].offset = dds_msg->fields()[i].offset();
      ros2_msg->fields[i].datatype = dds_msg->fields()[i].datatype();
      ros2_msg->fields[i].count = dds_msg->fields()[i].count();
    }

    ros2_msg->is_bigendian = dds_msg->is_bigendian();
    ros2_msg->point_step = dds_msg->point_step();
    ros2_msg->row_step = dds_msg->row_step();
    ros2_msg->data = dds_msg->data();
    ros2_msg->is_dense = dds_msg->is_dense();

    publisher_->publish(std::move(ros2_msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>
    lidar_subscriber_;
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <network_interface>\n";
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarToROS2Node>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
