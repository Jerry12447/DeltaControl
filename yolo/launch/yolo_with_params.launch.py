#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 宣告啟動參數
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='YOLO 置信度閾值'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/rgb',
        description='輸入影像話題'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/removed_cords',
        description='輸出座標話題'
    )
    
    execution_topic_arg = DeclareLaunchArgument(
        'execution_topic',
        default_value='delta_execution_complete',
        description='執行完成話題'
    )
    
    # YOLO 檢測節點
    yolo_node = Node(
        package='yolo',
        executable='yoloboundingbox',
        name='yolo_detection_node',
        output='screen',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'execution_complete_topic': LaunchConfiguration('execution_topic'),
        }],
        remappings=[
            ('/rgb', LaunchConfiguration('input_topic')),
            ('/removed_cords', LaunchConfiguration('output_topic')),
            ('delta_execution_complete', LaunchConfiguration('execution_topic'))
        ]
    )
    
    return LaunchDescription([
        confidence_arg,
        input_topic_arg,
        output_topic_arg,
        execution_topic_arg,
        yolo_node,
    ])
