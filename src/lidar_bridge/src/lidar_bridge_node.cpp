#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/idl/ros2/PointCloud2_.hpp"

#include <cmath>  // Added for std::sqrt and std::isfinite

class LidarToROS2Node : public rclcpp::Node
{
public:
  explicit LidarToROS2Node(const std::string & interface_name)
  : rclcpp::Node("lidar_bridge_node")
  {
    // Declare filtering parameters
    this->declare_parameter("enable_filtering", true);
    this->declare_parameter("min_radius", 0.3);
    this->declare_parameter("min_height", -10.0);
    this->declare_parameter("max_height", 10.0);

    enable_filtering_ = this->get_parameter("enable_filtering").as_bool();
    min_radius_ = this->get_parameter("min_radius").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();

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

    // Apply filtering if enabled
    if (enable_filtering_) {
      // Find offsets for x, y, z fields dynamically
      size_t offset_x = SIZE_MAX;
      size_t offset_y = SIZE_MAX;
      size_t offset_z = SIZE_MAX;
      for (const auto& field : ros2_msg->fields) {
        if (field.name == "x") offset_x = field.offset;
        else if (field.name == "y") offset_y = field.offset;
        else if (field.name == "z") offset_z = field.offset;
      }

      if (offset_x == SIZE_MAX || offset_y == SIZE_MAX || offset_z == SIZE_MAX) {
        RCLCPP_WARN(this->get_logger(), "Point cloud missing x, y, or z fields; publishing unfiltered.");
      } else {
        // Filter: remove NaN/invalid, near-origin, and vertical extremes
        std::vector<uint8_t> filtered_data;
        filtered_data.reserve(ros2_msg->data.size());

        const uint8_t* data_ptr = ros2_msg->data.data();
        size_t num_points = ros2_msg->width * ros2_msg->height;

        for (size_t i = 0; i < num_points; ++i) {
          size_t byte_idx = i * ros2_msg->point_step;

          const float x = *reinterpret_cast<const float*>(data_ptr + byte_idx + offset_x);
          const float y = *reinterpret_cast<const float*>(data_ptr + byte_idx + offset_y);
          const float z = *reinterpret_cast<const float*>(data_ptr + byte_idx + offset_z);

          if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            float range = std::sqrt(x * x + y * y + z * z);
            if (range >= min_radius_ && z >= min_height_ && z <= max_height_) {
              filtered_data.insert(filtered_data.end(), data_ptr + byte_idx, data_ptr + byte_idx + ros2_msg->point_step);
            }
          }
        }

        // Update message with filtered data
        if (!filtered_data.empty()) {
          ros2_msg->data = std::move(filtered_data);
          ros2_msg->width = ros2_msg->data.size() / ros2_msg->point_step;
          ros2_msg->row_step = ros2_msg->width * ros2_msg->point_step;
          ros2_msg->height = 1;  // Flatten to unorganized if necessary
          ros2_msg->is_dense = true;
        } else {
          RCLCPP_WARN(this->get_logger(), "All points filtered out; publishing empty cloud.");
        }
      }
    }

    publisher_->publish(std::move(ros2_msg));
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>
    lidar_subscriber_;

  // Filtering parameters
  bool enable_filtering_;
  double min_radius_;
  double min_height_;
  double max_height_;
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