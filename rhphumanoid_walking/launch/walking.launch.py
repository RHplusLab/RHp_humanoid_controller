import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory('rhphumanoid_walking'),
        'config',
        'rhphumanoid_walking_param.yaml'
    )

    return LaunchDescription([
        Node(
            package='rhphumanoid_walking',
            executable='rhphumanoid_walking_node',
            name='rhphumanoid_walking',
            output='screen',
            parameters=[{'walking_param_path': param_path}],
        )
    ])
