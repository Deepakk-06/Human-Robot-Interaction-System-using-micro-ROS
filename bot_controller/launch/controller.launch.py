"""
controller.launch.py  —  Gesture Controlled Robot (Upgraded)

Changes:
  - TimerAction delays spawners by 5 s so controller_manager has time to start
  - wheel_controller spawner uses --undefok to suppress spurious errors
  - Both spawners run concurrently after the delay (was sequential)
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--undefok', 'controller_manager_timeout',
        ],
        output='screen',
    )

    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'wheel_controller',
            '--controller-manager', '/controller_manager',
            '--undefok', 'controller_manager_timeout',
        ],
        output='screen',
    )

    # Delay both spawners by 5 s so Ignition Gazebo has time to
    # start the ign_ros2_control plugin and register the controller_manager.
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster, wheel_controller]
    )

    return LaunchDescription([delayed_spawners])
