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

"""Load the shared robot specification into the ROS C++ drive controller."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spec = json.loads((Path(get_package_share_directory('amr_description')) /
                       'config/compact_amr.json').read_text())
    wheels, motion, control = spec['wheels'], spec['motion_targets'], spec['control']
    parameters = dict(motion)
    parameters.pop('initial_test_speed_m_s')
    parameters.update(control)
    parameters.update(wheel_radius_m=wheels['radius_m'], wheelbase_m=wheels['wheelbase_m'],
                      track_m=wheels['track_m'], roller_handedness=wheels['roller_handedness'],
                      joint_velocity_limit_rad_s=wheels['joint_velocity_limit_rad_s'],
                      wheel_joint_names=[name + '_wheel_joint' for name in wheels['order']])
    return LaunchDescription([Node(package='my_amr', executable='my_amr_node',
                                   output='screen', parameters=[parameters])])
