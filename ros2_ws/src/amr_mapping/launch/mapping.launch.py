# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Run the installed SLAM Toolbox with an explicit scan topic and AMR TF owners."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Configure then activate native SLAM; reuse drive/odom/RSP through bringup."""
    package = Path(get_package_share_directory('amr_mapping'))
    bringup = Path(get_package_share_directory('my_amr')) / 'launch/bringup.launch.py'
    simulation = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': simulation,
            'scan_topic': LaunchConfiguration('scan_topic'),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan/front'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(package / 'config/slam.yaml')),
        DeclareLaunchArgument('start_bringup', default_value='true',
                              description='Start drive/odom/RSP; false if already running'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            condition=IfCondition(LaunchConfiguration('start_bringup')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': 'true', 'validation': 'false',
                'sim': LaunchConfiguration('sim'),
                'headless': LaunchConfiguration('headless'),
            }.items()),
        # Match the installed upstream launch's lifecycle startup sequence.
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE))])),
        slam,
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE)),
        Node(package='rviz2', executable='rviz2', name='mapping_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': simulation}],
             arguments=['-d', str(package / 'rviz/mapping.rviz')],
             remappings=[('/scan/front', LaunchConfiguration('scan_topic'))]),
    ])
