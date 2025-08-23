import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanQoSRelay(Node):
    def __init__(self):
        super().__init__('scan_qos_relay')

        # === CONFIGURE THE LISTENER (SUBSCRIBER) ===
        # This profile uses 'Best Effort' reliability to successfully listen to the
        # pointcloud_to_laserscan node's /scan topic.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === CONFIGURE THE SPEAKER (PUBLISHER) ===
        # This profile uses 'Reliable' reliability, which is what slam_toolbox and RViz expect.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === CREATE THE SUBSCRIBER AND PUBLISHER ===
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',  # Listen to the original topic
            self.scan_callback,
            qos_profile=sub_qos
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_reliable',  # Publish on a new, reliable topic
            qos_profile=pub_qos
        )

        self.get_logger().info('Scan QoS Relay is running: /scan (Best Effort) -> /scan_reliable (Reliable)')

    def scan_callback(self, msg):
        # Immediately republish the received message on the new topic
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