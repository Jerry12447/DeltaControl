from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('delta_robot_isaacsim'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='delta_robot_isaacsim',
            executable='trajectory_plan',
            name='trajectory_plan_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='delta_robot_isaacsim',
            executable='delta_control',
            name='delta_control_node',
            output='screen'
        )
    ])