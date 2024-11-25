import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class DronePublisher(Node):

    def __init__(self):
        super().__init__('drone_publisher')
        self.publisher_ = self.create_publisher(String, 'drone_topic', 10)
        timer_period = 1  # seconds 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello drone %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('publishing message %s' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    drone_publisher = DronePublisher()

    rclpy.spin(drone_publisher)

    drone_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
