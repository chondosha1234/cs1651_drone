import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from time import sleep 
from tf_transformations import euler_from_quaternion

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

        self.global_local_sub = self.create_subscription(
                Odometry,
                '/mavros/global_position/local',
                self.global_local_callback,
                qos_profile
        )

        self.drone_state_sub
        self.local_pos_sub
        self.global_pos_sub
        self.global_local_sub

    def local_pos_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        #self.get_logger().info(f"Position x: {position.x}, y: {position.y}, z: {position.z}")

    def global_pos_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        #self.get_logger().info(f"GPS lat: {latitude}, lon: {longitude}, alt: {altitude} ")
    
    def state_callback(self, msg):
        #self.get_logger().info(f"Drone mode: {msg.mode}")
        if msg.armed:
            self.get_logger().info("The drone is armed.")
    
    def global_local_callback(self, msg):
        poseX = msg.pose.pose.position.x
        poseY = msg.pose.pose.position.y
        poseZ = msg.pose.pose.position.z
        
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = math.degrees(yaw)

        self.get_logger().info(f"yaw: {yaw}")

        twistX = msg.twist.twist.linear.x
        twistY = msg.twist.twist.linear.y
        twistZ = msg.twist.twist.linear.z

        #self.get_logger().info("Pose x,y,z")
        #self.get_logger().info(f"\tx: {poseX}")
        #self.get_logger().info(f"\ty: {poseY}")
        #self.get_logger().info(f"\tz: {poseZ}")

        #self.get_logger().info("Twist x,y,z")
        #self.get_logger().info(f"\tx: {twistX}")
        #self.get_logger().info(f"\ty: {twistY}")
        #self.get_logger().info(f"\tz: {twistZ}")
        sleep(3)                     
    
def main(args=None):

    rclpy.init(args=args)
    node = DroneListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


