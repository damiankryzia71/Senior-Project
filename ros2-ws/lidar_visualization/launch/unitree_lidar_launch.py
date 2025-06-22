from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pcd_topic',
            default_value='/unilidar/cloud',
            description='Topic name for the LiDAR Pointcloud2 provider'
        ),

        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node'
        ),

        Node(
            package='lidar_visualization',
            executable='lidar_visualization_node',
            name='lidar_visualization_node',
            output='screen',
            parameters=[
                {'pcd_topic': LaunchConfiguration('pcd_topic')}
            ]
        )
    ])