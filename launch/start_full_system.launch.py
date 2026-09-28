#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches:
      1) 'bridge' node immediately
      2) after 2s => 'strongestMAC', 'horizontal_scan_node',
         'vertical_scan_node', 'robot_control_gui'
    """
    ld = LaunchDescription()

    # 1) Bridge node: start immediately
    bridge_node = Node(
        package='stacy_py',
        executable='bridge',
        name='bridge_node',
        output='log'
    )

    # 2) Group for the other nodes (launched together after 2s)
    other_nodes = GroupAction([
        Node(
            package='stacy_py',
            executable='strongestMAC',
            name='strongest_mac_node',
            output='log'
        ),
        Node(
            package='stacy_py',
            executable='horizontal_scan_node',
            name='horizontal_scan_node',
            output='log'
        ),
        Node(
            package='stacy_py',
            executable='vertical_scan_node',
            name='vertical_scan_node',
            output='log'
        ),
        Node(
            package='stacy_py',
            executable='robot_control_gui',
            name='robot_control_gui',
            output='log'
        ),
    ])

    # Delay launching the other_nodes by 2 seconds
    delayed_others = TimerAction(
        period=2.0,
        actions=[other_nodes]
    )

    ld.add_action(bridge_node)
    ld.add_action(delayed_others)

    return ld
