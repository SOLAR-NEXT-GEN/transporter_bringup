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
             'serial', '-b', '2000000', '--dev', '/dev/ttyTransporterMobile'],
        output='screen',
        emulate_tty=True,
    )

    mani_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '-b', '115200', '--dev', '/dev/ttyTransporterLeftMani'],
        output='screen',
        emulate_tty=True,
    )

    bno_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 
             'serial', '-b', '2000000', '--dev', '/dev/ttyTransporterBNO'],
        output='screen',
        emulate_tty=True,
    )

    gps = Node(
        package='sparkfun_rtk_express',
        executable='gps_publisher.py',
        name='gps_publisher',
        output='screen',
    )

    # yaw_reader = Node(
    #     package='hwt101ct_tilt_angle_sensor',
    #     executable='hwt101ct_yaw_publisher.py',
    #     name='hwt101ct_yaw_publisher',
    #     output='screen',
    #     remappings=[('hwt101ct_yaw_publisher', '/imu')],
    # )

    yaw_reader = Node(
        package='transporter_imu',
        executable='imu_converter.py',
        name='imu_converter',
        output='screen',
    )

    path_generator = Node(
        package='transporter_path',
        executable='straight_path_generator.py',
        name='straight_path_generator',
        output='screen',
        parameters=[{
            'path_length': 2.0,
            'waypoint_spacing': 0.05,
            'start_offset': 0.0,
        }]
    )
    
    navigation_scheduler = Node(
        package='transporter_bringup',
        executable='navigation_scheduler.py',
        name='navigation_scheduler',
        output='screen'
    )
    
    pure_pursuit = Node(
        package='transporter_controller',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'lookahead_distance': 0.5,
            'max_linear_velocity': 0.1225,
            'max_angular_velocity': 0.064
        }],
        remappings=[('/cmd_vel', '/cmd_vel/pure_pursuit')],
    )
    
    return LaunchDescription([
        twist_mux_node,
        joy_node,
        mobile_node,
        mani_node,
        bno_node,
        gps,
        yaw_reader,
        mani_control,
        transport_joy_node,
        path_generator,
        navigation_scheduler,
        pure_pursuit,
        localization,
        description,

    ])