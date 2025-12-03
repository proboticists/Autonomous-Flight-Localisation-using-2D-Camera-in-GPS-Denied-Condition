"""
Feature detector node (placeholder): subscribes to camera images, runs SuperPoint (or ORB fallback)
and publishes keypoints + descriptors. Right now it publishes a simplified message using
sensor_msgs/Image (with descriptors as numpy serialized in a stamped topic would be later).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np


class FeatureDetector(Node):
    def __init__(self):
        super().__init__('feature_detector')
        self.declare_parameter('input_topic', '/camera/image_raw')
        topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            Image, topic, self.image_cb, 10)
        self.pub = self.create_publisher(
            Image, '/vision/debug_keypoints_image', 10)
        self.br = CvBridge()
        # placeholder ORB for now
        self.detector = cv2.ORB_create(nfeatures=512)
        self.get_logger().info('FeatureDetector started, subscribing to: %s' % topic)

    def image_cb(self, msg: Image):
        img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kps = self.detector.detect(gray, None)
        out = cv2.drawKeypoints(
            img, kps, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        out_msg = self.br.cv2_to_imgmsg(out, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):

    rclpy.init(args=args)
    node = FeatureDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
