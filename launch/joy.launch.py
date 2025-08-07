#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )
    
    transport_joy_node = Node(
        package='transporter_joy',
        executable='joy.py',
        name='transporter_joy',
        output='screen',
    )
    
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '-b', '2000000', '--dev', '/dev/ttyTransporterMobile'],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        micro_ros_agent,
        joy_node,
        transport_joy_node,
    ])