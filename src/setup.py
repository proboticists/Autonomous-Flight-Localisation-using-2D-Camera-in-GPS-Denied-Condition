from setuptools import setup


package_name = 'vision_localization'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", ["launch/bringup_launch.py"]),
        ("share/" + package_name + "/config", ["config/default_params.yaml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'feature_detector = vision_localization.feature_detector_node:main',
            'feature_matcher = vision_localization.feature_matcher_node:main',
            'localization_fusion = vision_localization.localization_fusion_node:main',
            'map_preprocessor = vision_localization.map_preprocessor:main',
            'global_planner = vision_localization.planner_node:main',
            'path_follower = vision_localization.path_follower_node:main',
        ],
    },
)
