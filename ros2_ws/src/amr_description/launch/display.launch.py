# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Inspect geometry with artificial joint states; do not use for simulation TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('amr_description'))
    description = (package / 'urdf/compact_amr.urdf').read_text()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'rviz/compact_amr.rviz')]),
    ])
