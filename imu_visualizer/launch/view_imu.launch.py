"""
view_imu.launch.py  —  Gesture Controlled Robot (Upgraded)

Launches IMU-only visualisation pipeline:
  ESP32 → micro-ROS agent → /imu/data_raw
    → imu_smoother → imu_filter_madgwick → RViz2 (TF + IMU marker)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('imu_visualizer')

    rviz_cfg  = os.path.join(pkg, 'rviz',  'imu_config.rviz')
    urdf_path = os.path.join(pkg, 'urdf',  'imu_cube.urdf')

    gain_arg = DeclareLaunchArgument(
        'madgwick_gain', default_value='0.05',
        description='Madgwick filter convergence gain')

    # 1. Smoother
    smoother = Node(
        package='imu_visualizer',
        executable='imu_smoother',
        name='imu_smoother',
        parameters=[{'window_min': 5, 'window_max': 20,
                     'motion_low': 0.4, 'motion_high': 2.5,
                     'use_median': True}],
        output='screen',
    )

    # 2. Madgwick filter
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag':     False,
            'world_frame': 'odom',
            'publish_tf':  True,
            'gain':        LaunchConfiguration('madgwick_gain'),
            'zeta':        0.001,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_filtered'),
            ('/imu/data',     '/imu/data/filtered'),
        ],
        output='screen',
    )

    # 3. Robot state publisher (URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen',
    )

    # 4. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([gain_arg, smoother, madgwick, rsp, rviz])
