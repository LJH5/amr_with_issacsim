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

"""Compose drive, measured odometry and optional simulation or validation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start_simulator(context):
    """Run the existing SDK wrapper without mixing system ROS and SDK Python."""
    script = Path(LaunchConfiguration('sim_script').perform(context)).expanduser().resolve()
    if not script.is_file():
        raise RuntimeError('Isaac Sim run.sh not found; set sim_script:=/path/to/isaac_sim/run.sh')
    command = ['bash', str(script), '--control', 'ros']
    if IfCondition(LaunchConfiguration('headless')).evaluate(context):
        command.append('--headless')
    simulator = ExecuteProcess(cmd=command, cwd=str(script.parent), output='screen')
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='Isaac Sim exited'))])),
        simulator,
    ]


def generate_launch_description():
    """Select one controller owner and include the existing odometry launch."""
    package = Path(get_package_share_directory('my_amr'))
    launch = package / 'launch'
    # Find the sibling SDK folder in isolated/merged colcon installs. Copied
    # installations can supply sim_script explicitly; no build-time path is baked in.
    script = next((parent / 'isaac_sim/run.sh' for parent in package.parents
                   if (parent / 'isaac_sim/run.sh').is_file()), Path('isaac_sim/run.sh'))
    return LaunchDescription([
        DeclareLaunchArgument('validation', default_value='false',
                              description='Run automatic drive trials instead of manual control'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Start Isaac Sim using its existing run.sh'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the optional Isaac Sim process without its GUI'),
        DeclareLaunchArgument('sim_script', default_value=str(script),
                              description='Path to isaac_sim/run.sh for sim:=true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        OpaqueFunction(function=start_simulator,
                       condition=IfCondition(LaunchConfiguration('sim'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch / 'drive.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('validation'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch / 'odometry.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch / 'validation.launch.py')),
            condition=IfCondition(LaunchConfiguration('validation'))),
    ])
