#!/usr/bin/env python3
"""
Integration test that verifies EKF node can start with use_sim_time:=true.

This test catches the deadlock bug where wait_until_started() blocks
before the executor begins spinning.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import pathlib
import os
import sys
from launch_testing.legacy import LaunchTestService
from launch import LaunchService


def generate_launch_description():
    test_dir = pathlib.Path(__file__).resolve().parent

    # Clock publisher script
    clock_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(test_dir, 'test_clock_publisher.py')],
        name='clock_publisher',
        output='screen'
    )

    # Odometry publisher script
    odom_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(test_dir, 'test_odom_publisher.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        name='odom_publisher',
        output='screen'
    )

    # Minimal EKF configuration
    ekf_params = {
        'use_sim_time': True,
        'frequency': 30.0,
        'two_d_mode': True,
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        # Minimal sensor config - enable X position from odometry
        # This is enough for EKF to start publishing
        'odom0': '/odom',
        'odom0_config': [True, False, False,   # X, Y, Z position
                         False, False, False,   # roll, pitch, yaw
                         False, False, False,   # X, Y, Z velocity
                         False, False, False,   # roll, pitch, yaw velocity
                         False, False, False],  # X, Y, Z acceleration
    }

    # EKF node - with the bug, this will DEADLOCK in initialize()
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='screen'
    )

    return LaunchDescription([
        clock_publisher,
        odom_publisher,
        ekf_node,
    ])


def main(argv=sys.argv[1:]):
    """
    Main test function - verifies EKF publishes /odometry/filtered.

    If this times out, it indicates a deadlock.
    """
    ld = generate_launch_description()

    # Test script that checks if /odometry/filtered is published
    test_dir = pathlib.Path(__file__).resolve().parent
    test_script = os.path.join(test_dir, 'test_ekf_startup_checker.py')

    # Launch checker as ExecuteProcess with explicit use_sim_time param
    test_action = ExecuteProcess(
        cmd=['python3', test_script, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
