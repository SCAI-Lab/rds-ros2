import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rds_params = os.path.join(
        get_package_share_directory('rds_ros2'), 'config', 'rds_node_params.yaml'
    )
    # === RDS PIPELINE NODES ===
    
    # Transform publishers for RDS coordinate system
    tf_main_body_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_main_body_frame_broadcaster',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'main_body_frame',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ]
    )
    
    tf_rds_flip = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_rds_flip_broadcaster',
        arguments=[
            '--frame-id', 'main_body_frame',
            '--child-frame-id', 'rds_frame',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '-1.57'
        ]
    )

    # LiDAR merger
    laserscan_merger = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        parameters=[{
            'destination_frame': 'base_link',
            'cloud_destination_topic': '/rds/input/all/points',
            'scan_destination_topic': '/scan_multi',
            'laserscan_topics': '/scan_left /scan_right',
            'angle_min': -3.14,      
            'angle_max': 3.14,        
            'angle_increment': 0.00437,
            'scan_time': 0.0,
            'range_min': 0.1,
            'range_max': 3.0,
        }],
        output='screen'
    )

    # Voxel filtering
    voxel_filter = Node(
        package='rds_ros2',
        executable='voxel_filter_node',
        name='voxel_filter',
        parameters=[{
            'filter_field_name': 'z',
            'filter_limit_min': -1.0,
            'filter_limit_max': 1.0,
            'filter_limit_negative': False,
            'leaf_size': 0.1
        }],
        remappings=[
            ('input', '/rds/input/all/points'),
            ('output', '/rds/input/filtered/points')
        ]
    )

    # RDS controller
    rds_node = Node(
        package='rds_ros2',
        executable='rds_node',
        name='rds_node',
        output='screen',
        parameters=[rds_params]
    )
    
    # RDS command processor
    rds_command_node = Node(
        package='rds_ros2',
        executable='rds_nominal_command_node',
        name='rds_command_node',
        output='screen',
        remappings=[
            ('remote_cmd_vel', '/whill/joy_vel'),
            ('rds_modulated_cmd_vel', '/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        tf_main_body_frame,
        tf_rds_flip,
        laserscan_merger,
        voxel_filter,
        rds_node,
        rds_command_node,
    ])