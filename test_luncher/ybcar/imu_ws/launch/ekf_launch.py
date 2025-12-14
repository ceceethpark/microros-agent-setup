from launch import LaunchDescription
from launch_ros.actions import Node
import os

package_share_dir = os.path.dirname(__file__)
params_file = os.path.join(os.path.dirname(__file__), '..', 'params', 'ekf_params.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file]
        )
    ])
