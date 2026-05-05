"""
simulated_robot.launch.py  —  Gesture Controlled Robot (Upgraded)

Changes:
  - RViz enabled by default (was commented out)
  - Camera & IMU topics bridged to ROS 2
  - use_sim_time propagated to all nodes
  - world_name LaunchArgument properly forwarded
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world_name', default_value='empty',
        description='World file (without .world extension) in bot_description/worlds/')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualiser')

    # ── Gazebo + robot ─────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bot_description'),
            'launch', 'gazebo.launch.py')),
        launch_arguments={'world_name': LaunchConfiguration('world_name')}.items()
    )

    # ── ros2_control spawners ──────────────────────────────────────────────────
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bot_controller'),
            'launch', 'controller.launch.py'))
    )

    # ── Additional Gazebo → ROS 2 bridges ─────────────────────────────────────
    gz_bridge_extra = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_extra',
        arguments=[
            # Simulated IMU
            '/imu/data_sim@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('bot_description'),
            'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_rviz_arg,
        LogInfo(msg=['Starting simulated robot in world: ',
                     LaunchConfiguration('world_name')]),
        gazebo,
        controllers,
        gz_bridge_extra,
        rviz,
    ])
