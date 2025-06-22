from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Topic name for input image stream'
        ),
        DeclareLaunchArgument(
            'pcd_topic',
            default_value='/lidar/points',
            description='Topic name for input point cloud'
        ),
        DeclareLaunchArgument(
            'slam_vocab_path',
            default_value='home/rescue1/ros2-ws/src/orbslam3_ros2/ORBvoc.txt',
            description='Path to ORB-SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'slam_config_path',
            default_value='home/rescue1/ros2-ws/src/orbslam3_ros2/config.yaml',
            description='Path to ORB-SLAM3 configuration YAML'
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
