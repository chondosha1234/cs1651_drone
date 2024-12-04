import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State

class DroneListener(Node):

    def __init__(self):
        super().__init__('drone_listener')
        
        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
        )

        self.drone_state_sub = self.create_subscription(
                State,
                '/mavros/state',
                self.state_callback,
                qos_profile
        )

        self.local_pos_sub = self.create_subscription(
                Odometry,
                '/mavros/local_position/odom',
                self.local_pos_callback,
                qos_profile
        )

        self.global_pos_sub = self.create_subscription(
                NavSatFix,
                '/mavros/global_position/global',
                self.global_pos_callback,
                qos_profile
        )
        self.drone_state_sub
        self.local_pos_sub
        self.global_pos_sub

    def local_pos_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f"Position x: {position.x}, y: {position.y}, z: {position.z}")

    def global_pos_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        self.get_logger().info(f"GPS lat: {latitude}, lon: {longitude}, alt: {altitude} ")
    
    def state_callback(self,msg):
        if msg.armed:
            self.get_logger().info("The drone is armed.")


    
def main(args=None):

    rclpy.init(args=args)
    node = DroneListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

