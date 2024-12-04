import rclpy

from rclpy.node import Node
from mavros_msgs.srv import CommandTOL
from std_msgs.msg import Bool



class DroneNode(Node):

    def __init__(self):
        super().__init__('drone_node')

        # publisher to control drone movements 

        self.arming_pub = self.create_publisher(
                Bool,
                '/mavros/cmd/arming',
                10
        )
        self.arming_pub
    
    def arm_drone(self):
        arm_msg = Bool()
        arm_msg.data = True
        self.arming_pub.publish(arm_msg)

    def disarm_drone(self):
        arm_msg = Bool()
        arm_msg.data = False
        self.arming_pub.publish(arm_msg)


    def takeoff(self, height):
        # takeoff to given height and hold 
        # probably loop until odom reads certain height and then stop rising
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        response = takeoff_service(
                min_pitch = 0,
                yaw=0,
                latitude=0,
                longitude=0,
                altitude=height
        )
        if response.success:
            rospy.get_logger().info("Taking off.")
        else:
            rospy.get_logger().info("Failed to takeoff.")


    def land(self):
        # return to 0 odometry -- or call land topic 
        land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        response = land_service(
                min_pitch=0,
                yaw=0,
                latitude=0,
                longitude=0,
                altitude=0
        )
        if response.success:
            rospy.get_logger().info("Landing...")
        else:
            rospy.get_logger().info("Failed to start landing.")


def main(args=None):

    rclpy.init(args=args)
    node = DroneNode()

    node.arm_drone()
    node.takeoff(2)
    node.land()
    node.disarm_drone()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



