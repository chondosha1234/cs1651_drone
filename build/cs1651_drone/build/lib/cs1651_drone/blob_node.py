import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_raw_sub = self.create_subscription(
                Image,
                '/image_raw',
                self.image_raw_callback,
                10
        )
        self.image_raw_sub
        self.bridge = CvBridge()

        # openCv blob detection?
        self.params = cv2.SimpleBlobDetector_Params()
        # config params if needed
        self.params.filterByColor = True
        self.params.blobColor = 255
        self.params.filterByArea = True
        self.params.minArea = 100
        self.params.maxArea = 10000
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.5
        self.detector = cv2.SimpleBlobDetector_create(self.params)


    def image_raw_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(e)

        #cv2.imshow('Frame', frame)
        #cv2.waitKey(1)

        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #_, threshold = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_color = (0, 100, 100)
        upper_color = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        keypoints = self.detector.detect(mask)

        for keypoint in keypoints:
            x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
            cv2.circle(cv_image, (x,y), int(keypoints.size/2), (0, 255, 0), 2)

        cv2.imshow('Blobs', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
