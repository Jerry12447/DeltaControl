from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 取得套件路徑
    package_share_dir = get_package_share_directory('plant_detection')
    
    # 預設配置檔案路徑
    default_config_file = os.path.join(
        package_share_dir,
        'config',
        'params.yaml'
    )
    
    # 宣告啟動參數
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='植物檢測節點配置檔案路徑'
    )
    
    # 使用 LaunchConfiguration 來取得參數值
    config_file = LaunchConfiguration('config_file')
    
    # 創建植物檢測節點
    plant_detection_video_node = Node(
        package='plant_detection',
        executable='plant_detection_video_node',
        name='plant_detection_video_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    plant_detection_node = Node(
        package='plant_detection',
        executable='plant_detection_node',
        name='plant_detection_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        plant_detection_node
    ])

