import rclpy
from rclpy.node import Node
#from pynput import keyboard
from std_msgs.msg import String
from geometry_msgs.msg import Point
from time import sleep

class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
            
        self.control_pub = self.create_publisher(
            String,
            '/drone_control',
            10
        )   

        self.point_pub = self.create_publisher(
                Point,
                '/points',
                10
        )

        """
        self.apriltag_sub = self.create_subscription(
                PoseStamped,
                '/apriltag_detections',
                self.apriltag_callback,
                10
        )
        self.apriltag_sub
        """
        #self.current_state = 'IDLE'
        #self.current_apriltag_pose = None

        #listener = keyboard.Listener(on_press=self.on_key_press)
        #listener.start()

    """

    
    def apriltag_callback(self, msg):
        return
    """

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    
    msg = String()

    key = ''
    while key != 'e':
        key = input("Enter command:")

        if key == 't':
            msg.data = "takeoff"
            node.control_pub.publish(msg)

        elif key == 'm':
            x = input("Enter x coordinate:")
            y = input("Enter y coordinate:")
            z = input("Enter z coordinate:")
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
            node.point_pub.publish(point)
            sleep(1)

            msg.data = "move"
            node.control_pub.publish(msg)

        elif key == 'f':
            msg.data = "follow"
            node.control_pub.publish(msg)

        elif key == 's':
            msg.data = "stop"
            node.control_pub.publish(msg)

        elif key == 'r':
            msg.data = "return"
            node.control_pub.publish(msg)

        elif key == 'l':
            msg.data = "land"
            node.control_pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


