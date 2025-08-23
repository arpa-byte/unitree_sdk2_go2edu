import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanQoSRelay(Node):
    def __init__(self):
        super().__init__('scan_qos_relay')

        # Subscribe to Best Effort /scan from pointcloud_to_laserscan
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publish Reliable /scan_reliable for slam_toolbox and RViz
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile=sub_qos
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_reliable',
            qos_profile=pub_qos
        )

        self.get_logger().info('Scan QoS Relay: /scan (BestEffort) -> /scan_reliable (Reliable)')

    def scan_cb(self, msg: LaserScan):
        # CRITICAL: ensure the stamp matches current ROS time
        msg.header.stamp = self.get_clock().now().to_msg()

        # Keep frame_id consistent (in case upstream leaves it empty)
        if not msg.header.frame_id:
            msg.header.frame_id = 'utlidar_lidar'

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanQoSRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
