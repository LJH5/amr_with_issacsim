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

"""Fuse encoder body velocities and IMU yaw/rate; EKF owns the odom TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep raw odometry separate and publish one fused odom-to-base transform."""
    package = Path(get_package_share_directory('my_amr'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    config = LaunchConfiguration('localization_params')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('localization_params',
                              default_value=str(package / 'config/localization.yaml')),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/odometry.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'publish_tf': 'false', 'odom_topic': '/odometry/wheel'}.items())]),
        Node(package='my_amr', executable='imu_covariance', output='screen',
             parameters=[config, clock]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter',
             output='screen', parameters=[config, clock, {
                 'publish_tf': ParameterValue(
                     LaunchConfiguration('publish_tf'), value_type=bool)}],
             remappings=[('odometry/filtered', '/odom'), ('reset', '/ekf_filter/reset')]),
    ])
