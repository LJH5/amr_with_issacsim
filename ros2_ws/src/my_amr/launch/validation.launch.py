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

"""Start the C++ controller and a finite low-speed measurement suite."""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Read canonical wheel geometry and expose the trial parameters."""
    wheels = json.loads((Path(get_package_share_directory('amr_description')) /
                         'config/compact_amr.json').read_text())['wheels']
    geometry = {'wheel_radius_m': wheels['radius_m'], 'wheelbase_m': wheels['wheelbase_m'],
                'track_m': wheels['track_m'], 'roller_handedness': wheels['roller_handedness'],
                'joint_velocity_limit_rad_s': wheels['joint_velocity_limit_rad_s'],
                'wheel_joint_names': [name + '_wheel_joint' for name in wheels['order']]}
    arguments = [('repetitions', '3', int), ('drive_seconds', '5.0', float),
                 ('settle_seconds', '2.0', float), ('speed_m_s', '0.2', float),
                 ('yaw_rate_rad_s', '0.3', float), ('travel_limit_m', '1.5', float),
                 ('stop_linear_tolerance_m_s', '0.02', float),
                 ('stop_yaw_tolerance_rad_s', '0.03', float),
                 ('stop_wheel_tolerance_rad_s', '0.2', float), ('settled_window_s', '0.5', float),
                 ('position_wrap_period_rad', str(4 * math.pi), float)]
    declarations = [DeclareLaunchArgument(name, default_value=default)
                    for name, default, _ in arguments]
    geometry.update({name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                     for name, _, kind in arguments})
    package = Path(get_package_share_directory('my_amr'))
    report_directory = Path((package / 'drive_report_directory.txt').read_text().strip())
    run_directory = report_directory / datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = str(run_directory / 'drive_validation')
    declarations.append(DeclareLaunchArgument(
        'output_prefix', default_value=prefix,
        description='Run directory and file prefix. Raw files go under report_data.'))
    declarations.append(DeclareLaunchArgument(
        'summary_markdown', default_value='true', description='Write a Markdown summary on exit'))
    drive = package / 'launch/drive.launch.py'

    def start_validation(context):
        output = Path(LaunchConfiguration('output_prefix').perform(context))
        if not output.name:
            raise ValueError('output_prefix must include a file prefix')
        if output.parent.name != 'report_data':
            output = output.parent / 'report_data' / output.name
        validation = Node(package='my_amr', executable='drive_validation', output='screen',
                          parameters=[{**geometry, 'output_prefix': str(output)}])
        summarize = IfCondition(LaunchConfiguration('summary_markdown')).evaluate(context)

        def on_validation_exit(_event, _context):
            if summarize and output.with_name(output.name + '_summary.csv').is_file():
                logger = get_logger('drive_validation_report')
                try:
                    functions = runpy.run_path(str(package / 'validation_report.py'))
                    write_report = functions['write_report']
                    report = write_report(output.parent.parent, prefix=output.name)
                    logger.info(f'Report: {report.resolve()}')
                except (OSError, csv.Error, ValueError) as error:
                    logger.warning(f'Could not write Markdown summary: {error}')
            return [EmitEvent(event=Shutdown(reason='Validation finished'))]

        return [RegisterEventHandler(OnProcessExit(
            target_action=validation, on_exit=on_validation_exit)), validation]

    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(drive))),
        OpaqueFunction(function=start_validation)])
