#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 獲取功能包路徑
    pkg_share = FindPackageShare(package='yolo').find('yolo')
    
    # 宣告啟動參數
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('yolo'),
            'config',
            'yolo_params.yaml'
        ]),
        description='YOLO 參數配置文件路徑'
    )
    
    # YOLO 檢測節點
    yolo_node = Node(
        package='yolo',
        executable='yoloboundingbox',
        name='yolo_detection_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('/rgb', '/rgb'),
            ('/removed_cords', '/removed_cords'),
            ('delta_execution_complete', 'delta_execution_complete')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        yolo_node,
    ])
