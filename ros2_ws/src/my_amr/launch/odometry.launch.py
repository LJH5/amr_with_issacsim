# Copyright 2026 LJH5
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Measured wheel odometry and robot TF; run alongside the drive controller."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Load canonical geometry and publish a single odom-to-URDF-root transform."""
    description = Path(get_package_share_directory('amr_description'))
    package = Path(get_package_share_directory('my_amr'))
    wheels = json.loads((description / 'config/compact_amr.json').read_text())['wheels']
    simulation = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    geometry = {'wheel_radius_m': wheels['radius_m'], 'wheelbase_m': wheels['wheelbase_m'],
                'track_m': wheels['track_m'], 'roller_handedness': wheels['roller_handedness'],
                'joint_velocity_limit_rad_s': wheels['joint_velocity_limit_rad_s'],
                'wheel_joint_names': [name + '_wheel_joint' for name in wheels['order']],
                'use_sim_time': simulation,
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(package / 'config/odometry.yaml')),
        Node(package='my_amr', executable='wheel_odometry', output='screen',
             parameters=[LaunchConfiguration('params_file'), geometry]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': (description / 'urdf/compact_amr.urdf').read_text(),
                          'use_sim_time': simulation}]),
    ])
