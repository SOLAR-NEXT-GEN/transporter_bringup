#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

datum_lat = 13.763656
datum_lon = 100.527998

def generate_launch_description():

    simulation_share  = get_package_share_directory('transporter_simulation')
    localization_share = get_package_share_directory('transporter_localization')
    bringup_share = get_package_share_directory('transporter_bringup')

    map_path = os.path.join(bringup_share, 'maps', 'track.osm')


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
            'start_offset': 1.0,
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
        }]
    )

    return LaunchDescription(
        [
            simulation,
            localization,
            joy_control,
            joy,
            path_generator,
            path_scheduler,
            pure_pursuit
        ]
    )
