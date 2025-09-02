import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('utlidar_launcher')
    go2_localization_pkg = get_package_share_directory('go2_localization') # Get path to new package

    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_config.rviz')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_mapper_params.yaml')
    pcl2scan_params = os.path.join(pkg_dir, 'config', 'pointcloud_to_laserscan_params.yaml')

    network_interface_arg = DeclareLaunchArgument(
        'network_interface', default_value='eno1',
        description='The network interface for DDS'
    )

    lidar_bridge_node = Node(
        package='lidar_bridge', executable='lidar_bridge_node',
        name='lidar_bridge_node', output='screen',
        arguments=[LaunchConfiguration('network_interface')]
    )

    # Modified odom_bridge_node: It now publishes raw odom and imu, but does NOT publish TF.
    odom_bridge_node = Node(
        package='odom_bridge', executable='odom_bridge_node',
        name='odom_bridge_node', output='screen',
        arguments=[LaunchConfiguration('network_interface')],
        parameters=[{'publish_tf': False}] # CRITICAL: Disable TF publishing
    )

    # --- NEW: Launch the EKF from our go2_localization package ---
    # This node will now be responsible for publishing the odom -> base_link transform.
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_localization_pkg, 'launch', 'ekf.launch.py')
        )
    )
    
    base_to_lidar_tf_node = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.15', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'utlidar_lidar']
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/lidar/point_cloud'), ('scan', '/scan')],
        parameters=[pcl2scan_params]
    )

    qos_relay_node = Node(
        package='qos_relay', executable='scan_relay_node', name='scan_qos_relay'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params],
        remappings=[('scan', '/scan_reliable'), ('odom', '/odometry/filtered')] # Use the filtered odom
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        network_interface_arg,
        lidar_bridge_node,
        odom_bridge_node,
        ekf_launch, # ADDED
        base_to_lidar_tf_node,
        pointcloud_to_laserscan_node,
        qos_relay_node,
        slam_toolbox_node,
        rviz_node
    ])