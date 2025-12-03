"""
Global planner skeleton using a simple grid and A* placeholder.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Planner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.path_pub = self.create_publisher(Path, '/planner/global_path', 10)
        self.get_logger().info('Planner started')


def publish_dummy_path(self):
    path = Path()
    path.header.frame_id = 'map'
    for i in range(5):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(i)
        p.pose.position.y = 0.0
        p.pose.orientation.w = 1.0
        path.poses.append(p)
    self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        import time
        while rclpy.ok():
            node.publish_dummy_path()
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
