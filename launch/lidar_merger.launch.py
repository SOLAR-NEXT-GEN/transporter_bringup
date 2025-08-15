#!/usr/bin/env python3
import os

from flask import config
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_share = get_package_share_directory('transporter_bringup')
    
    ros2_laser_scan_merger_config = os.path.join(bringup_share, 'config', 'ros2_laser_scan_merger_config.yaml')
    description_share = get_package_share_directory('transporter_description')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch/transporter_rviz.launch.py'))
    )

    lidar_frontleft = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='lidar_frontleft',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyLidarFrontLeft',
                         'serial_baudrate': 460800,
                         'frame_id': 'lidar_frontleft',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            remappings=[('/scan', '/lidar_frontleft')],
            output='screen')

    lidar_frontright = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='lidar_frontright',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyLidarFrontRight',
                         'serial_baudrate': 460800,
                         'frame_id': 'lidar_frontright',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Standard'}],
            remappings=[('/scan', '/lidar_frontright')],
            output='screen')
    
    ros2_laser_scan_merger_node = Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[ros2_laser_scan_merger_config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        )

    pointcloud_to_laserscan_node = Node(
            name='pointcloud_to_laserscan',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[ros2_laser_scan_merger_config]
        )
        
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    
    # Transporter joy node
    transport_joy_node = Node(
        package='transporter_joy',
        executable='joy.py',
        name='transporter_joy',
        output='screen',
    )
    
    # micro-ROS agent
    mobile_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '-b', '2000000', '--dev', '/dev/ttyTransporterMobile'],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        lidar_frontleft,
        lidar_frontright,
        pointcloud_to_laserscan_node,
        ros2_laser_scan_merger_node,
        joy_node,
        mobile_node,
        transport_joy_node,
        description,
    ])

#ros2 bag record -a
