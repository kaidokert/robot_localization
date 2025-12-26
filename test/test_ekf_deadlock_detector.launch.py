#!/usr/bin/env python3
"""
Deadlock detector test - verifies EKF doesn't hang during initialize() with use_sim_time.

This test specifically replicates the deadlock scenario:
1. Start EKF node FIRST (before clock publisher)
2. EKF calls wait_until_started() and blocks
3. Start clock publisher AFTER (too late - executor not spinning yet)
4. Verify EKF either: starts successfully (with fix) or times out (with bug)
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import pathlib
import os
import sys
from launch_testing.legacy import LaunchTestService
from launch import LaunchService


def generate_launch_description():
    test_dir = pathlib.Path(__file__).resolve().parent

    # EKF configuration
    ekf_params = {
        'use_sim_time': True,
        'frequency': 30.0,
        'two_d_mode': True,
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        'odom0': '/odom',
        'odom0_config': [True, False, False,
                         False, False, False,
                         False, False, False,
                         False, False, False,
                         False, False, False],
    }

    # Start EKF node FIRST - with the bug, this will block in initialize()
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen'
    )

    # Start clock publisher AFTER a delay (simulates starting clock later)
    # With the bug: EKF is already deadlocked, so clock won't help
    # With the fix: EKF starts, then receives clock once it's spinning
    clock_publisher = TimerAction(
        period=1.0,  # Start clock 1 second later
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(test_dir, 'test_clock_publisher.py')],
                name='clock_publisher',
                output='screen'
            )
        ]
    )

    # Odometry publisher also starts after delay
    odom_publisher = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(test_dir, 'test_odom_publisher.py'),
                     '--ros-args', '-p', 'use_sim_time:=true'],
                name='odom_publisher',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ekf_node,         # Starts immediately
        clock_publisher,  # Starts after 1 second
        odom_publisher,   # Starts after 1 second
    ])


def main(argv=sys.argv[1:]):
    """
    Main test function - verifies EKF publishes /odometry/filtered.

    With the bug: EKF deadlocks in initialize() before clock starts, test times out
    With the fix: EKF starts successfully, receives clock once spinning, test passes
    """
    ld = generate_launch_description()

    # Test script that checks if /odometry/filtered is published
    test_dir = pathlib.Path(__file__).resolve().parent
    test_script = os.path.join(test_dir, 'test_ekf_startup_checker.py')

    # Give EKF 15 seconds to start (with fix: fast, with bug: times out)
    test_action = TimerAction(
        period=2.0,  # Wait 2 seconds before starting checker
        actions=[
            ExecuteProcess(
                cmd=['python3', test_script, '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
            )
        ]
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
