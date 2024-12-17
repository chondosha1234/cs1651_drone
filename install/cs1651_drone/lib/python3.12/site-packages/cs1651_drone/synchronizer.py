import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Header

class SynchronizedImagePublisher(Node):
    def __init__(self):
        super().__init__('synchronized_image_publisher')

        self.image_sub = Subscriber(self, Image, '/image_raw')
        self.camera_info_sub = Subscriber(self, CameraInfo, '/camera_info')

        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.camera_info_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        self.synchronized_image_pub = self.create_publisher(
                Image,
                '/image_rect',
                10
        )

    def callback(self, image_msg, camera_info_msg):
        self.synchronized_image_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedImagePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
