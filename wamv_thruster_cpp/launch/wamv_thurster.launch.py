from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg = 'wamv_thruster_cpp'
    share = get_package_share_directory(pkg)

    ekf_local_yaml  = os.path.join(share, 'config', 'ekf_local.yaml')
    ekf_global_yaml  = os.path.join(share, 'config', 'ekf_global.yaml')
    navsat_yaml     = os.path.join(share, 'config', 'navsat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # EKF LOCAL (IMU -> /odometry/filtered, frames odom->base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf_local_yaml, {'use_sim_time': use_sim_time}],
            remappings=[]
        ),

        # NAVSAT (GPS+IMU+odom -> /odometry/gps)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_yaml, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/wamv/sensors/imu/imu/data'),
                ('gps/fix', '/wamv/sensors/gps/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),
            ]
        ),

        # EKF GLOBAL (IMU + /odometry/gps -> /odometry/filtered_map, frames map->base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[ekf_global_yaml, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/gps', '/odometry/gps'),
            ]
        ),

        # SEU NÓ DE THRUSTER
        Node(
            package=pkg,
            executable='thruster_node',  # <-- ajuste para o nome do executável instalado pelo CMake
            name='thruster_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])