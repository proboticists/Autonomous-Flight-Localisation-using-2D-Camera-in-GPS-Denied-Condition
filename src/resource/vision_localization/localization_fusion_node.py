"""
Localization fusion node (skeleton). Subscribes to /localization/pose2d and IMU/altitude and publishes fused_pose.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu


class LocalizationFusion(Node):
    def __init__(self):
        super().__init__('localization_fusion')
        self.pose_sub = self.create_subscription(
            PoseStamped, '/localization/pose2d', self.pose_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_cb, 10)
        self.fused_pub = self.create_publisher(
            PoseStamped, '/localization/fused_pose', 10)
        self.last_pose = None

    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg
        # naive passthrough for now
        self.fused_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
