from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
         # 引数の定義
        DeclareLaunchArgument(
            'output', default_value='log', description='screen or log'
        ),
        DeclareLaunchArgument(
            'frame_id', default_value='map', description='Frame ID'
        ),
        DeclareLaunchArgument(
            'map_reso', default_value='0.05', description='Map resolution'
        ),
        DeclareLaunchArgument(
            'map_size', default_value='30.0', description='Map size'
        ),
        
         # raycast_mapping_node の起動
        Node(
            package='raycast_mapping_ros',
            executable='raycast_mapping_node',
            name='raycast_mapping',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'map_reso': LaunchConfiguration('map_reso'),
                'map_size': LaunchConfiguration('map_size'),
            }]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '1', '0','/map', '/base_link']
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/user/.rviz2/raycast_mapping.rviz'],
        )
    ])
