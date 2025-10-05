from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Simple launch file to test RDS with simulation."""
    
    return LaunchDescription([
        # Transform publishers (base_link → main_body_frame → rds_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_main_body_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'main_body_frame'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rds_frame',
            arguments=['0', '0', '0', '-1.57', '0', '0', 'main_body_frame', 'rds_frame'],
            output='screen'
        ),
        
        # Point cloud concatenation
        Node(
            package='rds_ros2',
            executable='pointcloud_concatenate.py',
            name='pc_concat',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'clouds': 2,
                'hz': 20
            }],
            remappings=[
                ('cloud_in1', '/hokuyo_left/out'),
                ('cloud_in2', '/hokuyo_right/out'),
                ('cloud_out', '/rds/input/all/points')
            ]
        ),
        
        # Voxel filtering
        Node(
            package='rds_ros2',
            executable='voxel_filter_node',
            name='voxel_filter',
            output='screen',
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
        
        # RDS Node
        Node(
            package='rds_ros2',
            executable='rds_node',
            name='rds_node',
            output='screen'
        ),
        
        # RDS Command Node
        Node(
            package='rds_ros2',
            executable='rds_nominal_command_node',
            name='rds_command_node',
            output='screen'
        ),
    ])