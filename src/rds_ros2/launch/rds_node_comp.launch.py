from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for RDS system."""
    
    # Declare any launch arguments here if needed
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add static transform publishers (these work similarly in ROS2)
    tf_rds_flip_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_rds_flip_broadcaster',
        arguments=['0', '0', '0', '-1.57', '0', '0', 'main_body_frame', 'rds_frame']
    )
    
    tf_main_body_frame_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_main_body_frame_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'main_body_frame']
    )
    
    # Python nodes for scan to pointcloud conversion
    # In ROS2, we usually define Python nodes differently, but for migration purposes
    # let's assume we've updated the Python script to be compatible with ROS2
    left_scan_points = Node(
        package='rds_ros',
        executable='scan_to_pointcloud.py',
        name='left_scan_points',
        parameters=[{
            'scan_topic': '/scan_left',
            'cloud_topic': '/scan/left/points'
        }],
        output='screen'
    )
    
    right_scan_points = Node(
        package='rds_ros',
        executable='scan_to_pointcloud.py',
        name='right_scan_points',
        parameters=[{
            'scan_topic': '/scan_right',
            'cloud_topic': '/scan/right/points'
        }],
        output='screen'
    )
    
    # Point cloud concatenation node
    pc_concat = Node(
        package='pointcloud_concatenate',
        executable='pointcloud_concatenate_node',
        name='pc_concat',
        parameters=[{
            'target_frame': 'base_link',
            'clouds': 2,
            'hz': 20
        }],
        remappings=[
            ('cloud_in1', '/scan/left/points'),
            ('cloud_in2', '/scan/right/points'),
            ('cloud_out', '/rds/input/all/points')
        ],
        output='screen'
    )
    
    # RDS nodes
    rds = Node(
        package='rds_ros',
        executable='rds_ros_node',
        name='rds'
    )
    
    command = Node(
        package='rds_ros',
        executable='rds_ros_nominal_command_node',
        name='command',
        output='screen'
    )
    
    # Here's the ROS2 component container that replaces the nodelet manager
    # This is where we'll load the PCL filter component
    component_container = ComposableNodeContainer(
        name='pcl_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # VoxelGrid filter component
            ComposableNode(
                package='pcl_ros',  # You'll need to check the actual ROS2 package name
                plugin='pcl_ros::VoxelGrid',  # Check the actual component name
                name='voxel_grid',
                remappings=[
                    ('input', '/rds/input/all/points'),
                    ('output', '/rds/input/filtered/points')
                ],
                parameters=[{
                    'filter_field_name': 'z',
                    'filter_limit_min': -1.0,
                    'filter_limit_max': 1.0,
                    'filter_limit_negative': False,
                    'leaf_size': 0.2
                }]
            )
        ],
        output='screen'
    )
    
    # Add all nodes to the launch description
    ld.add_action(tf_rds_flip_broadcaster)
    ld.add_action(tf_main_body_frame_broadcaster)
    ld.add_action(left_scan_points)
    ld.add_action(right_scan_points)
    ld.add_action(pc_concat)
    ld.add_action(rds)
    ld.add_action(command)
    ld.add_action(component_container)
    
    return ld