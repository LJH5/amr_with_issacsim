# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Saved-map AMCL and the native Nav2 servers needed for point-to-point navigation."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
import yaml


def start_navigation(context):
    """Resolve map assets and shared footprint, then connect Twist through Nav2's filters."""
    package = Path(get_package_share_directory('amr_navigation'))
    description = Path(get_package_share_directory('amr_description'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise RuntimeError(f'Saved map YAML not found: {map_file}; supply map:=/path/map.yaml')
    metadata = yaml.safe_load(map_file.read_text())
    if not isinstance(metadata, dict) or not metadata.get('image'):
        raise RuntimeError(f'Map YAML requires an image entry: {map_file}')
    image = (map_file.parent / metadata['image']).expanduser().resolve()
    if not image.is_file():
        raise RuntimeError(f'Map image not found: {image}')
    params_file = Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve()
    if not params_file.is_file():
        raise RuntimeError(f'Nav2 parameters not found: {params_file}')
    spec = json.loads((description / 'config/compact_amr.json').read_text())
    length, width = spec['design']['footprint_size_m']
    footprint = [[length / 2, width / 2], [length / 2, -width / 2],
                 [-length / 2, -width / 2], [-length / 2, width / 2]]
    rewrites = {
        'footprint': json.dumps(footprint),
        'map_server.ros__parameters.yaml_filename': str(map_file),
        'amcl.ros__parameters.scan_topic': LaunchConfiguration('scan_topic'),
    }
    for scope in ('local_costmap', 'global_costmap'):
        for sensor, argument in (('front', 'scan_topic'), ('rear', 'rear_scan_topic')):
            key = f'{scope}.{scope}.ros__parameters.obstacle_layer.{sensor}.topic'
            rewrites[key] = LaunchConfiguration(argument)
    for sensor, argument in (('front', 'scan_topic'), ('rear', 'rear_scan_topic')):
        rewrites[f'collision_monitor.ros__parameters.{sensor}.topic'] = LaunchConfiguration(argument)
    configured = RewrittenYaml(
        source_file=str(params_file), param_rewrites=rewrites, convert_types=True).perform(context)
    simulation = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    # Reuse Nav2's localization lifecycle management and AMCL /initialpose support.
    actions = [GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2 / 'launch/localization_launch.py')),
        launch_arguments={
            'map': str(map_file), 'params_file': configured,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true', 'use_composition': 'False',
        }.items())])]
    # These are installed Nav2 nodes, without unused docking/route/waypoint servers.
    servers = (
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_planner', 'planner_server', []),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
    )
    for node_package, executable, remappings in servers:
        actions.append(Node(
            package=node_package, executable=executable, name=executable, output='screen',
            parameters=[configured, {'use_sim_time': simulation}], remappings=remappings))
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'use_sim_time': simulation,
                     'node_names': [executable for _, executable, _ in servers]}]))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='navigation_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', str(package / 'rviz/navigation.rviz')],
        parameters=[{'use_sim_time': simulation}],
        remappings=[('/scan/front', LaunchConfiguration('scan_topic')),
                    ('/scan/rear', LaunchConfiguration('rear_scan_topic'))]))
    return actions


def generate_launch_description():
    """Start the robot stack optionally, and keep AMCL as the sole map-to-odom owner."""
    package = Path(get_package_share_directory('amr_navigation'))
    mapping = Path(get_package_share_directory('amr_mapping'))
    robot = Path(get_package_share_directory('my_amr'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(mapping / 'map/map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(package / 'config/nav2.yaml')),
        DeclareLaunchArgument('scan_topic', default_value='/scan/front'),
        DeclareLaunchArgument('rear_scan_topic', default_value='/scan/rear'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_bringup', default_value='true'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot / 'launch/bringup.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_bringup')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': 'true', 'validation': 'false',
                'params_file': str(robot / 'config/odometry.yaml'),
                'sim': LaunchConfiguration('sim'), 'headless': LaunchConfiguration('headless'),
            }.items())]),
        OpaqueFunction(function=start_navigation),
    ])
