from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """Launch RDS with related nodes."""
    
    return LaunchDescription([
        # Static TF publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rds_flip_broadcaster',
            arguments=['0', '0', '0', '-1.57', '0', '0', 'main_body_frame', 'rds_frame']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_main_body_frame_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'main_body_frame']
        ),
        
        # Scan to pointcloud converters
        Node(
            package='rds_ros2',  
            executable='scan_to_pointcloud.py',
            name='left_scan_points',
            output='screen',
            parameters=[{
                'scan_topic': '/scan_left',
                'cloud_topic': '/scan/left/points'
            }]
        ),
        
        Node(
            package='rds_ros2',  
            executable='scan_to_pointcloud.py',
            name='right_scan_points',
            output='screen',
            parameters=[{
                'scan_topic': '/scan_right',
                'cloud_topic': '/scan/right/points'
            }]
        ),
        
        # Pointcloud concatenation
        Node(
            package='rds_ros2', 
            executable='pointcloud_concatenate.py',  # Use our custom script
            name='pc_concat',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'clouds': 2,
                'hz': 20
            }],
            remappings=[
                ('cloud_in1', '/scan/left/points'),
                ('cloud_in2', '/scan/right/points'),
                ('cloud_out', '/rds/input/all/points')
            ]
        ),
        
        # RDS Node
        Node(
            package='rds_ros2',
            executable='rds_node',
            name='rds'
        ),
        
        # Command Node
        Node(
            package='rds_ros2',
            executable='rds_nominal_command_node',  
            name='command',
            output='screen'
        ),
        
        # PCL processing using components (replaces nodelets)
        Node(
            package='rds_ros2',
            executable='voxel_filter_node',
            name='voxel_grid',
            parameters=[{
                'filter_field_name': 'z',
                'filter_limit_min': -1.0,
                'filter_limit_max': 1.0,
                'filter_limit_negative': False,
                'leaf_size': 0.2
            }],
            remappings=[
                ('input', '/rds/input/all/points'),
                ('output', '/rds/input/filtered/points')
            ]
        ),
    ])