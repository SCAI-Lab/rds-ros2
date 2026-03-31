from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    RDS Core Processing Launch File
    
    Starts the core RDS system for point cloud processing and obstacle avoidance:
    - Transform publishers for coordinate frames
    - Point cloud concatenation (Python script)
    - Voxel grid filtering (C++)  
    - RDS obstacle avoidance node (C++)
    - RDS velocity modulator (C++)
    
    No teleop or GUI - those are started separately.
    """
    
    return LaunchDescription([
        
        # =============================================================================
        # COORDINATE TRANSFORMS
        # =============================================================================
        
        # Transform: base_link → main_body_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_main_body_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'main_body_frame'],
            output='screen'
        ),
        
        # Transform: main_body_frame → rds_frame (rotated -90° around Y-axis)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            name='tf_rds_frame',
            arguments=['0', '0', '0', '0', '-1.57', '0', 'main_body_frame', 'rds_frame'],
            output='screen'
        ),
        
        # =============================================================================
        # POINT CLOUD PROCESSING PIPELINE
        # =============================================================================
        
        # Point cloud concatenation (Python script - since C++ version is empty)
        Node(
            package='rds_ros2',
            executable='pointcloud_concatenate.py',
            name='pc_concatenate',
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
        
        # Voxel grid filtering (C++)
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
        
        # =============================================================================
        # RDS CORE SYSTEM
        # =============================================================================
        
        # Main RDS node - provides obstacle avoidance service (C++)
        Node(
            package='rds_ros2',
            executable='rds_node',
            name='rds_node',
            output='screen'
        ),
        
        # RDS velocity modulator - integrates commands with obstacle avoidance (C++)
        Node(
            package='rds_ros2',
            executable='rds_nominal_command_node',
            name='rds_velocity_modulator',
            output='screen',
            remappings=[
                ('remote_cmd_vel', '/cmd_vel_teleop'),
                ('/rds_modulated_cmd_vel', '/rds_modulated_cmd_vel')
            ]
        ),
        
    ])