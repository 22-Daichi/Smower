import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        self.get_logger().info(f'Received image at time {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()