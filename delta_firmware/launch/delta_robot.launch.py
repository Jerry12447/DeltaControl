from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('delta_firmware'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='delta_firmware',
            executable='trajectory_plan',
            name='trajectory_plan_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='delta_firmware',
            executable='delta_firmware_control',
            name='delta_control_node',
            output='screen'
        ),
        Node(
            package='delta_firmware',
            executable='delta_firmware_api',
            name='delta_firmware_api_node',
            output='screen'
        )
    ])