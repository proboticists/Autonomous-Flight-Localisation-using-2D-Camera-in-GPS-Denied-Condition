"""
Minimal ROS2 launch to bring up core nodes for testing (Python launch).
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


ld = LaunchDescription()


ld.add_action(Node(
    package='vision_localization',
    executable='feature_detector',
    name='feature_detector',
    output='screen'))


ld.add_action(Node(
    package='vision_localization',
    executable='feature_matcher',
    name='feature_matcher',
    output='screen'))


ld.add_action(Node(
    package='vision_localization',
    executable='localization_fusion',
    name='localization_fusion',
    output='screen'))


ld.add_action(Node(
    package='vision_localization',
    executable='global_planner',
    name='global_planner',
    output='screen'))


ld.add_action(Node(
    package='vision_localization',
    executable='path_follower',
    name='path_follower',
    output='screen'))


return ld
