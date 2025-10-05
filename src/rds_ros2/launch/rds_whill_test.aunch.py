from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Simple launch file to test RDS with simulation."""
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_main_body_frame_broadcaster',
            arguments=[
                '--frame-id', 'base_link',
                '--child-frame-id', 'main_body_frame',
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0'
            ],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rds_flip_broadcaster',
            arguments=[
                '--frame-id', 'main_body_frame',
                '--child-frame-id', 'rds_frame',
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '-1.57',
                '--pitch', '0',
                '--yaw', '0'
            ],
            output='screen'
        ),


        Node(
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
                'range_max': 4.0,
            }],
            output='screen',
        ),

        
        # # # Point cloud concatenation
        # Node(
        #     package='rds_ros2',
        #     executable='pointcloud_concatenate.py',
        #     name='pc_concat',
        #     output='screen',
        #     parameters=[{
        #         'target_frame': 'base_link',
        #         'clouds': 2,
        #         'hz': 20
        #     }],
        #     remappings=[
        #         ('cloud_in1', '/scan/left/points'),
        #         ('cloud_in2', '/scan/right/points'),
        #         ('cloud_out', '/rds/input/all/points')
        #     ]
        # ),
        
        # # Voxel filtering
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
                'leaf_size': 0.02
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
            output='screen',
            remappings=[
                ('remote_cmd_vel', '/whill/joy_vel'),           
                ('rds_modulated_cmd_vel', '/cmd_vel')       
            ]
        ),
    ])