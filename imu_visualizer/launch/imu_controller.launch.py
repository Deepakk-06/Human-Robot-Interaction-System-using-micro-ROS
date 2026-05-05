"""
imu_controller.launch.py  —  Gesture Controlled Robot (Upgraded)

Launch graph:
  /imu/data_raw  →  [imu_smoother]  →  /imu/data_filtered
                                     ↓
                            [imu_filter_madgwick]  →  /imu/data/filtered
                                                     ↓
                                           [imu_gesture_controller]
                                             ↓                ↓
                                      /cmd_vel   /wheel_controller/cmd_vel_unstamped
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    speed_mode_arg = DeclareLaunchArgument(
        'speed_mode', default_value='NORMAL',
        description='Initial speed mode: SLOW | NORMAL | FAST')

    madgwick_gain_arg = DeclareLaunchArgument(
        'madgwick_gain', default_value='0.05',
        description='Madgwick filter gain (lower = smoother, higher = faster)')

    # ── 1. IMU smoother (adaptive median + moving-average) ────────────────────
    imu_smoother = Node(
        package='imu_visualizer',
        executable='imu_smoother',
        name='imu_smoother',
        parameters=[{
            'window_min':  5,
            'window_max':  20,
            'motion_low':  0.4,
            'motion_high': 2.5,
            'use_median':  True,
        }],
        remappings=[
            ('/imu/data_raw',      '/imu/data_raw'),
            ('/imu/data_filtered', '/imu/data_filtered'),
        ],
        output='screen',
    )

    # ── 2. Madgwick orientation filter ────────────────────────────────────────
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'use_mag':     False,
            'world_frame': 'odom',
            'publish_tf':  True,
            'gain':        LaunchConfiguration('madgwick_gain'),
            # Prevents drift accumulation on stationary robot
            'zeta':        0.001,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_filtered'),
            ('/imu/data',     '/imu/data/filtered'),
        ],
        output='screen',
    )

    # ── 3. Gesture → Twist controller ────────────────────────────────────────
    imu_controller = Node(
        package='imu_visualizer',
        executable='imu_controller',
        name='imu_gesture_controller',
        parameters=[{
            'speed_mode':        LaunchConfiguration('speed_mode'),
            'pitch_deadband':    0.08,
            'yaw_deadband':      0.12,
            'pitch_full_scale':  0.628,   # ~36°
            'yaw_full_scale':    1.047,   # ~60°
            'ramp_rate':         3.0,     # m/s per second
            'estop_pitch_delta': 0.55,    # rad sudden change
            'estop_lockout_s':   1.5,
            'watchdog_s':        0.5,
            'publish_rate_hz':   50.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        speed_mode_arg,
        madgwick_gain_arg,
        LogInfo(msg=['Launching IMU controller in speed_mode=',
                     LaunchConfiguration('speed_mode')]),
        imu_smoother,
        imu_filter,
        imu_controller,
    ])
