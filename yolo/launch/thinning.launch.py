#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 獲取功能包路徑
    pkg_share = get_package_share_directory('yolo')
    
    # 參數文件路徑
    config_file = os.path.join(pkg_share, 'config', 'thinning_params.yaml')
    
    # 宣告啟動參數
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/rgb',
        description='輸入影像話題'
    )
    
    # 整合疏苗節點
    thinning_node = Node(
        package='yolo',
        executable='thinning_algorithm',
        name='thinning_node',
        parameters=[config_file, {
            'input_topic': LaunchConfiguration('input_topic')
        }],
        output='screen'
    )

    single_frame_detection_node = Node(
        package='yolo',
        executable='single_frame_detection',
        name='single_frame_detection_node',
        parameters=[config_file, {
            'input_topic': LaunchConfiguration('input_topic')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        #thinning_node,
        single_frame_detection_node
    ])
