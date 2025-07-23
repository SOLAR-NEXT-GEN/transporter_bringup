#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_share = get_package_share_directory('transporter_bringup')
    
    twist_mux_config = os.path.join(bringup_share, 'config', 'ros2_twist_mux.yaml')
    localization_share = get_package_share_directory('transporter_localization')
    description_share = get_package_share_directory('transporter_description')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch/localization.launch.py'))
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch/transporter_rviz.launch.py'))
    )

    twist_mux_node = Node(
        package='ros2_twist_mux',
        executable='ros2_twist_mux.py',
        name='ros2_twist_mux',
        parameters=[twist_mux_config],
        output='screen',
        emulate_tty=True,
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
        remappings=[('/cmd_vel', '/cmd_vel/joy')],
    )

    mani_control = Node(
        package='transporter_joy',
        executable='mani_control.py',
        name='mani_control',
        output='screen',
    )
    
    # micro-ROS agent
    mobile_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '-b', '115200', '--dev', '/dev/ttyTransporterMobile'],
        output='screen',
        emulate_tty=True,
    )

    # mani_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
    #          'serial', '-b', '115200', '--dev', '/dev/ttyTransporterLeftMani'],
    #     output='screen',
    #     emulate_tty=True,
    # )

    gps = Node(
        package='sparkfun_rtk_express',
        executable='gps_publisher.py',
        name='gps_publisher',
        output='screen',
    )

    yaw_reader = Node(
        package='hwt101ct_tilt_angle_sensor',
        executable='hwt101ct_yaw_publisher.py',
        name='hwt101ct_yaw_publisher',
        output='screen',
        remappings=[('hwt101ct_yaw_publisher', '/imu')],
    )

    path_generator = Node(
        package='transporter_controller',
        executable='straight_path_generator.py',
        name='straight_path_generator',
        output='screen',
        parameters=[{
            'path_length': 4.0,
            'waypoint_spacing': 0.1,
            'start_offset': 1.0,
        }]
    )
    
    path_scheduler = Node(
        package='transporter_controller',
        executable='path_real_planning.py',
        name='path_scheduler',
        output='screen'
    )
    
    pure_pursuit = Node(
        package='transporter_controller',
        executable='diff_pure_pursit.py',
        name='diff_pure_pursuit_path_follower',
        output='screen',
        parameters=[{
            'lookahead_distance': 3.0,
            'max_linear_vel': 0.1225,
            'max_angular_vel': 0.064
        }],
        remappings=[('/cmd_vel', '/cmd_vel/pure_pursuit')],
    )
    
    return LaunchDescription([
        twist_mux_node,
        mobile_node,
        # mani_node,
        joy_node,
        mani_control,
        transport_joy_node,
        path_generator,
        path_scheduler,
        pure_pursuit,
        # localization,
        # description,
        # gps,
        # yaw_reader,
    ])