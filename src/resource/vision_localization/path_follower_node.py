"""
Local path follower skeleton: subscribes to /planner/global_path and publishes TwistStamped velocity commands.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path_sub = self.create_subscription(
            Path, '/planner/global_path', self.path_cb, 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

    def path_cb(self, msg: Path):
        # simple: publish small forward velocity to demo wiring
        twist = TwistStamped()
        twist.twist.linear.x = 0.5
        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
