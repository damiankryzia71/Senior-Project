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
            'left_topic',
            default_value='/camera/camera/infra1/image_rect_raw',
            description='Left infrared image topic'
        ),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/camera/camera/infra2/image_rect_raw',
            description='Right infrared image topic'
        ),
        DeclareLaunchArgument(
            'slam_vocab_path',
            default_value='/home/rescue1/ros2-ws/src/orbslam3_ros2/ORBvoc.txt',
            description='Path to ORB-SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'slam_config_path',
            default_value='/home/rescue1/ros2-ws/src/orbslam3_ros2/stereo_realsense_config.yaml',
            description='Path to ORB-SLAM3 stereo configuration YAML'
        ),
        DeclareLaunchArgument(
            'with_viewer',
            default_value='true',
            description='Enable ORB-SLAM3 viewer window'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_depth': 'false',
                'enable_color': 'false'
            }.items()
        ),

        Node(
            package='orbslam3_ros2',
            executable='track_stereo_node',
            name='track_stereo_node',
            output='screen',
            parameters=[
                {'left_topic': LaunchConfiguration('left_topic')},
                {'right_topic': LaunchConfiguration('right_topic')},
                {'slam_vocab_path': LaunchConfiguration('slam_vocab_path')},
                {'slam_config_path': LaunchConfiguration('slam_config_path')},
                {'with_viewer': LaunchConfiguration('with_viewer')}
            ]
        )
    ])
