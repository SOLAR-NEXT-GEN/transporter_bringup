#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


datum_lat = 13.763656
datum_lon = 100.527998

def generate_launch_description():

    simulation_share  = get_package_share_directory('transporter_simulation')
    localization_share = get_package_share_directory('transporter_localization')
    bringup_share = get_package_share_directory('transporter_bringup')
    
    twist_mux_config = os.path.join(bringup_share, 'config', 'ros2_twist_mux.yaml')
    
    twist_mux_node = Node(
        package='ros2_twist_mux',
        executable='ros2_twist_mux.py',
        name='ros2_twist_mux',
        parameters=[twist_mux_config],
        output='screen',
        emulate_tty=True,
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulation_share, 'launch/gazebo.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(localization_share, 'launch/localization_sim.launch.py'))
    )

    joy_control = Node(
        package='transporter_simulation',
        executable='sim_joy.py',
        name='joy_control',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel/joy')],
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )

    path_generator = Node(
        package='transporter_controller',
        executable='straight_path_generator.py',
        name='straight_path_generator',
        output='screen',
        parameters=[{
            'path_length': 12.0,
            'waypoint_spacing': 0.1,
            'start_offset': 0.0,
        }]
    )
    
    path_scheduler = Node(
        package='transporter_controller',
        executable='path_scheduler.py',
        name='path_scheduler',
        output='screen'
    )
    
    pure_pursuit = Node(
        package='transporter_controller',
        executable='diff_pure_pursit.py',
        name='diff_pure_pursuit_path_follower',
        output='screen',
        parameters=[{
            'lookahead_distance': 1.0,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0
        }],
        remappings=[('/cmd_vel', '/cmd_vel/pure_pursuit')],
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn Left Hinge controller
    left_hinge_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_hinge_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn Right Hinge controller
    right_hinge_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_hinge_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_left_hinge_controller = TimerAction(
        period=4.0,
        actions=[left_hinge_controller_spawner],
    )

    delayed_right_hinge_controller = TimerAction(
        period=5.0,
        actions=[right_hinge_controller_spawner],
    )

    mani_sim = Node(
        package='transporter_simulation',
        executable='sim_mani.py',
        name='mani_sim',
        output='screen',
        parameters=[{
            'left_hinge_up': 0.0,
            'left_hinge_down': -2.17,
            'right_hinge_up': 0.0,
            'right_hinge_down': 2.17,
            'animation_step': 0.05,
            'animation_rate': 20,
            'animation_duration': 2.0,
        }],
    )

    return LaunchDescription(
        [
            simulation,
            localization,
            twist_mux_node,
            joy_control,
            joy,
            path_generator,
            path_scheduler,
            pure_pursuit,
            mani_sim,
            delayed_joint_state_broadcaster,
            delayed_left_hinge_controller,
            delayed_right_hinge_controller,
        ]
    )
