import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist   #maybe not necessary
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
        )

        self.subscription = self.create_subscription(
                Image,
                '/image_raw',
                self.listener_callback,
                10
        )

        self.global_pos_sub = self.create_subscription(
                Odometry,
                '/mavros/global_position/local',
                self.position_callback,
                qos_profile
        )

        self.detection_sub = self.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.detection_callback,
                10
        )
        self.subscription
        self.detection_sub
        self.global_pos_sub

        self.bridge = CvBridge()

        self.target_detected = False
        self.target_centre = None    # notice spelling of centre *

        self.camera_center_x = 640 // 2
        self.camera_center_y = 480 // 2
        
        self.yaw = 0.0
        self.threshold = 20

    def listener_callback(self, msg):
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.target_detected and self.target_centre:
            cv2.circle(current_frame, (int(self.target_centre.x), int(self.target_centre.y)), 10, (0, 0))

        cv2.imshow('Camera Feed', current_frame)
        cv2.waitKey(1)

    def position_callback(self, msg):
        orientation = msg.pose.pose.orientation
        (_,_,yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = math.degrees(yaw)
        self.yaw = yaw

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
            #print("[CAMERA_NODE]: Detected an april tag!")
            
            offset_x = self.target_centre.x - self.camera_center_x
            offset_y = self.target_centre.y - self.camera_center_y
            
            (offset_x, offset_y) = self.transform_coordinates(offset_x, offset_y)


            if abs(offset_x) > self.threshold:
                if offset_x > 0:
                    print("Move right (positive x)")
                else:
                    print("Move left (negative x)")
            
            if abs(offset_y) > self.threshold:
                if offset_y > 0:
                    print("Move down (negative y)")
                else:
                    print("move up (positive y)")

        else:
            self.target_detected = False
            #print("[CAMERA_NODE]: No apriltag detected.")

    def transform_coordinates(self, offset_x, offset_y):

        # rotation matrix
        matrix = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],
                           [np.sin(self.yaw), np.cos(self.yaw)]])

        # transfrom camera orientation to drone frame of reference 
        new_offsets = np.dot(matrix, np.array([offset_x, offset_y]))
        return new_offsets[0], new_offsets[1]

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
