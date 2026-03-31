import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rds_params = os.path.join(
        get_package_share_directory('rds_ros2'), 'config', 'rds_node_params.yaml'
    )

    return LaunchDescription([

        # === TF: base_link -> main_body_frame -> rds_frame ===
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_main_body_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'main_body_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rds_frame',
            arguments=['0', '0', '0', '-1.57', '0', '0', 'main_body_frame', 'rds_frame'],
        ),

        # === Fake LiDAR ===
        Node(
            package='rds_ros2',
            executable='fake_laser_publisher.py',
            name='fake_laser',
            output='screen',
            parameters=[{
                'left_scan_topic': '/scan_left',
                'right_scan_topic': '/scan_right',
                'rate': 10.0,
                'simulate_obstacles': True,
            }]
        ),

        # === Scan -> PointCloud2 (one per scanner) ===
        Node(
            package='rds_ros2',
            executable='scan_to_pointcloud.py',
            name='scan_to_cloud_left',
            output='screen',
            parameters=[{
                'scan_topic': '/scan_left',
                'cloud_topic': '/scan/left/points',
            }]
        ),
        Node(
            package='rds_ros2',
            executable='scan_to_pointcloud.py',
            name='scan_to_cloud_right',
            output='screen',
            parameters=[{
                'scan_topic': '/scan_right',
                'cloud_topic': '/scan/right/points',
            }]
        ),

        # === Concatenate point clouds ===
        Node(
            package='rds_ros2',
            executable='pointcloud_concatenate.py',
            name='pc_concat',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'clouds': 2,
                'hz': 20,
            }],
            remappings=[
                ('cloud_in1', '/scan/left/points'),
                ('cloud_in2', '/scan/right/points'),
                ('cloud_out', '/rds/input/all/points'),
            ]
        ),

        # === Voxel filter ===
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
                'leaf_size': 0.1,
            }],
            remappings=[
                ('input', '/rds/input/all/points'),
                ('output', '/rds/input/filtered/points'),
            ]
        ),

        # === RDS node ===
        Node(
            package='rds_ros2',
            executable='rds_node',
            name='rds_node',
            output='screen',
            parameters=[rds_params]
        ),

        # === RDS client (calls service at 10 Hz, reads from /cmd_vel_in) ===
        Node(
            package='rds_ros2',
            executable='rds_client_ros_node.py',
            name='rds_client',
            output='screen',
            remappings=[
                ('cmd_vel_in', '/cmd_vel_in'),
                ('rds_modulated_cmd_vel', '/cmd_vel'),
            ]
        ),

        # === Fake pedestrian tracks ===
        # Pedestrian 1: walks straight toward robot from ahead
        # Pedestrian 2: crosses left-to-right at 2 m ahead
        # Pedestrian 3: orbits at 3 m radius
        Node(
            package='rds_ros2',
            executable='fake_pedestrian_publisher.py',
            name='fake_pedestrians',
            output='screen',
            parameters=[{
                'topic': 'rds/input/pedestrian_tracks',
                'rate': 10.0,
                'pedestrian_radius': 0.3,
            }]
        ),

        # === GUI ===
        Node(
            package='rds_gui_ros2',
            executable='rds_gui_node',
            name='rds_gui',
            output='screen',
        ),
    ])

