import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
#from mavros_msgs.msg import SetMode
from nav_msgs.msg import Odometry

class DronePublisher(Node):

    def __init__(self):
        super().__init__('drone_publisher')

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
        )

        self.arming_pub = self.create_publisher(
                Bool,
                '/mavros/cmd/arming',
                10
        )
    
        self.subscription = self.create_subscription(
                Odometry,
                '/mavros/local_position/odom',
                self.odom_callback,
                qos_profile
        )
        self.subscription

        self.arm_drone()
    
    def arm_drone(self):
        arm_msg = Bool()
        arm_msg.data = True
        self.arming_pub.publish(arm_msg)
        self.get_logger().info("arming drone")

    def odom_callback(self, msg):
        self.get_logger().info("starting odom callback")
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.get_logger().info(
                f"Position x: {position.x}, y: {position.y}, z: {position.z} | "
                f"Orientation x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DronePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


