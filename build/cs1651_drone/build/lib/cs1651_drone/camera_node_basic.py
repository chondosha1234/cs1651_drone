import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist   #maybe not necessary
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
                Image,
                '/image_raw',
                self.listener_callback,
                10
        )

        self.detection_sub = self.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.detection_callback,
                10
        )
        self.subscription
        self.detection_sub
        self.bridge = CvBridge()

        self.target_detected = False
        self.target_centre = None

    def listener_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.target_detected and self.target_centre:
            cv2.circle(current_frame, (int(self.target_centre.x), int(self.target_centre.y)), 10, (0, 0))

        cv2.imshow('Camera Feed', current_frame)
        cv2.waitKey(1)

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
            print("[CAMERA_NODE]: Detected an april tag!")
        else:
            self.target_detected = False
            print("[CAMERA_NODE]: No apriltag detected.")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
