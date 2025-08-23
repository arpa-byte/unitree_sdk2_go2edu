import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('utlidar_launcher')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam_config.rviz')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_mapper_params.yaml')
    pcl2scan_params = os.path.join(pkg_dir, 'config', 'pointcloud_to_laserscan_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='eno1',
        description='The network interface for DDS'
    )

    lidar_bridge_node = Node(
        package='lidar_bridge',
        executable='lidar_bridge_node',
        name='lidar_bridge_node',
        output='screen',
        arguments=[LaunchConfiguration('network_interface')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    #Static transforms
    odom_to_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    base_to_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.15', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'utlidar_lidar'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/lidar/point_cloud'), ('scan', '/scan')],
        parameters=[pcl2scan_params, {'use_sim_time': use_sim_time}]
    )

    qos_relay_node = Node(
        package='qos_relay',
        executable='scan_relay_node',
        name='scan_qos_relay',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        network_interface_arg,
        lidar_bridge_node,
        odom_to_base_tf_node,
        base_to_lidar_tf_node,
        pointcloud_to_laserscan_node,
        qos_relay_node,
        slam_toolbox_node,
        rviz_node
    ])
