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
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Topic name for input image stream'
        ),
        DeclareLaunchArgument(
            'pcd_topic',
            default_value='/unilidar/cloud',
            description='Topic name for input point cloud'
        ),
        DeclareLaunchArgument(
            'slam_vocab_path',
            default_value='/home/rescue1/ros2-ws/src/orbslam3_ros2/ORBvoc.txt',
            description='Path to ORB-SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'slam_config_path',
            default_value='/home/rescue1/ros2-ws/src/orbslam3_ros2/rgbl_config.yaml',
            description='Path to ORB-SLAM3 configuration YAML'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file)
        ),

        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            parameters=[
                {'range_min': 0.5},
                {'range_max': 20}
            ]
        ),

        Node(
            package='orbslam3_ros2',
            executable='track_rgbl_node',
            name='track_rgbl_node',
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'pcd_topic': LaunchConfiguration('pcd_topic')},
                {'slam_vocab_path': LaunchConfiguration('slam_vocab_path')},
                {'slam_config_path': LaunchConfiguration('slam_config_path')},
            ]
        )
    ])
