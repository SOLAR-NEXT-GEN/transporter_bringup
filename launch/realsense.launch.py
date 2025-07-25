from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_pkg_share = FindPackageShare('realsense2_camera').find('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    
    bringup_share = get_package_share_directory('transporter_bringup')
    
    rviz_config_file = os.path.join(bringup_share, 'rviz', 'rs.rviz')
    
    unite_imu_method_arg = DeclareLaunchArgument(
        'unite_imu_method',
        default_value='2',
        description='Method to unite IMU data: 0-None, 1-Copy, 2-Linear interpolation'
    )
    
    enable_gyro_arg = DeclareLaunchArgument(
        'enable_gyro',
        default_value='true',
        description='Enable gyroscope'
    )
    
    enable_accel_arg = DeclareLaunchArgument(
        'enable_accel',
        default_value='true',
        description='Enable accelerometer'
    )
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'enable_gyro': LaunchConfiguration('enable_gyro'),
            'enable_accel': LaunchConfiguration('enable_accel')
        }.items()
    )
    
    move_forward_node = Node(
        package='transporter_controller',
        executable='calculate_dist.py',
        name='realsense_move_forward',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        unite_imu_method_arg,
        enable_gyro_arg,
        enable_accel_arg,
        realsense_launch,
        move_forward_node,
        rviz_node
    ])