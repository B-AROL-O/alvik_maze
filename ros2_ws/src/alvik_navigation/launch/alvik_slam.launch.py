#!/usr/bin/env python3
"""
Alvik SLAM Launch File
Starts all necessary nodes for mapping and navigation:
- ToF receiver (LaserScan publisher)
- Odometry publisher (odom->base_link TF)
- SLAM Toolbox (online async mapping)
- Foxglove Bridge (visualization)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('alvik_navigation')

    # Launch arguments
    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='192.168.1.152',
        description='MQTT broker host address'
    )

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # SLAM config file path
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')

    # ToF Receiver Node - subscribes to MQTT and publishes LaserScan
    tof_receiver_node = Node(
        package='alvik_navigation',
        executable='tof_receiver',
        name='tof_receiver',
        output='screen',
        parameters=[{
            'mqtt_host': LaunchConfiguration('mqtt_host'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
        }]
    )

    # Odometry Publisher Node - subscribes to MQTT and publishes Odometry + TF
    odom_publisher_node = Node(
        package='alvik_navigation',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{
            'mqtt_host': LaunchConfiguration('mqtt_host'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
        }]
    )

    # SLAM Toolbox - async online mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Foxglove Bridge - WebSocket bridge for visualization
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'num_threads': 4,
        }]
    )

    return LaunchDescription([
        mqtt_host_arg,
        mqtt_port_arg,
        use_sim_time_arg,
        tof_receiver_node,
        odom_publisher_node,
        slam_toolbox_node,
        foxglove_bridge_node,
    ])
