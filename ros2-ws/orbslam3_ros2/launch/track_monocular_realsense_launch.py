from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'topic_name',
            default_value='/camera/camera/color/image_raw',
            description='Topic name for input image stream'
        ),
        DeclareLaunchArgument(
            'slam_vocab_path',
            default_value='home/rescue1/ros2-ws/src/orbslam3_ros2/ORBvoc.txt',
            description='Path to ORB-SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'slam_config_path',
            default_value='home/rescue1/ros2-ws/src/orbslam3_ros2/monocular_config.yaml',
            description='Path to ORB-SLAM3 configuration YAML'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file)
        ),

        Node(
            package='orbslam3_ros2',
            executable='track_monocular_node',
            name='track_monocular_node',
            parameters=[
                {'topic_name': LaunchConfiguration('topic_name')},
                {'slam_vocab_path': LaunchConfiguration('slam_vocab_path')},
                {'slam_config_path': LaunchConfiguration('slam_config_path')}
            ]
        )
    ])