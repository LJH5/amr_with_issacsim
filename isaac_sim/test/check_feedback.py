#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""USD/installed-node-schema integration checks without starting physics or DDS."""

import csv
import json
import os
from pathlib import Path
import re
import sys
import subprocess
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from feedback import GRAPH_PATH, configure_feedback, create_imu_sensor, find_feedback_prims
from check_topics import imu_values, wheel_values


SPEC = json.loads((SIM_DIR.parent / 'ros2_ws/src/amr_description/config/compact_amr.json').read_text())
SDK = SIM_DIR.parents[1] / 'IsaacSim'


class FeedbackChecks(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self.robot = '/World/compact_mecanum_amr'
        root = UsdGeom.Xform.Define(self.stage, self.robot + '/base_footprint').GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)
        imu = UsdGeom.Xform.Define(self.stage, self.robot + '/imu_link').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(imu)
        for name in SPEC['wheels']['order']:
            UsdPhysics.RevoluteJoint.Define(self.stage, self.robot + '/joints/' + name + '_wheel_joint')
        self.modules = {name: ModuleType(name) for name in (
            'omni', 'omni.graph', 'omni.graph.core', 'omni.kit', 'omni.kit.commands')}
        for name, module in self.modules.items():
            if '.' in name:
                parent, child = name.rsplit('.', 1)
                setattr(self.modules[parent], child, module)
        self.controller = SimpleNamespace(
            create_graph=Mock(side_effect=self.create_graph),
            create_node=Mock(side_effect=self.create_node),
            attribute=self.attribute,
            connect=Mock(side_effect=lambda source, dest: dest.AddConnection(source.GetPath())),
            set=lambda attribute, value: attribute.Set(value))
        self.modules['omni.graph.core'].Controller = self.controller
        self.modules['omni.graph.core'].GraphPipelineStage = SimpleNamespace(
            GRAPH_PIPELINE_STAGE_SIMULATION='simulation')
        self.graph_api = Mock()
        self.graph_api.get_pipeline_stage.return_value = 'simulation'
        self.modules['omni.graph.core'].get_graph_by_path = Mock(return_value=self.graph_api)
        self.execute = Mock(side_effect=self.execute_sensor)
        self.modules['omni.kit.commands'].execute = self.execute

    def create_graph(self, options):
        path = options['graph_path']
        self.assertFalse(self.stage.GetPrimAtPath(path), 'Attempted to recreate existing graph')
        self.stage.DefinePrim(path, 'OmniGraph')

    def create_node(self, path, node_type):
        node = self.stage.DefinePrim(path, 'OmniGraphNode')
        node.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(node_type)
        extension, name = node_type.rsplit('.', 1)
        doc = SDK / 'exts' / extension / 'docs/ogn' / ('Ogn' + name + '.rst')
        if node_type == 'omni.graph.action.OnPlaybackTick':
            doc = next((SDK / 'extscache').glob('omni.graph.action_nodes-*/ogn/docs/OgnOnPlaybackTick.rst'))
        self.assertTrue(doc.is_file(), f'Node is not in installed 5.1 SDK: {node_type}')
        types = {'execution': Sdf.ValueTypeNames.Token, 'bool': Sdf.ValueTypeNames.Bool,
                 'string': Sdf.ValueTypeNames.String, 'double': Sdf.ValueTypeNames.Double,
                 'float': Sdf.ValueTypeNames.Float, 'uint64': Sdf.ValueTypeNames.UInt64,
                 'uchar': Sdf.ValueTypeNames.UChar, 'vectord[3]': Sdf.ValueTypeNames.Double3,
                 'quatd[4]': Sdf.ValueTypeNames.Quatd}
        for line in doc.read_text().splitlines():
            if not line.startswith('    "'):
                continue
            fields = next(csv.reader([line.strip()], skipinitialspace=True))
            attribute = re.search(r'\(\*((?:inputs|outputs):\w+)\*\)', fields[0])
            if not attribute:
                continue
            type_name = fields[1].strip('`')
            if type_name == 'target':
                node.CreateRelationship(attribute.group(1))
            elif type_name in types:
                node.CreateAttribute(attribute.group(1), types[type_name])

    def attribute(self, path):
        attribute = self.stage.GetAttributeAtPath(path)
        self.assertTrue(attribute, f'Attribute is absent from installed SDK node schema: {path}')
        return attribute

    def execute_sensor(self, command, **kwargs):
        self.assertEqual(command, 'IsaacSensorCreateImuSensor')
        prim = self.stage.DefinePrim(kwargs['parent'] + kwargs['path'], 'IsaacImuSensor')
        prim.CreateAttribute('sensorPeriod', Sdf.ValueTypeNames.Double)
        prim.CreateAttribute('enabled', Sdf.ValueTypeNames.Bool)
        return True, prim

    def test_native_feedback_wiring_and_idempotency(self):
        old_graph = self.stage.DefinePrim('/World/ActionGraph', 'OmniGraph')
        with patch.dict(sys.modules, self.modules):
            configure_feedback(self.stage, SPEC)
            configure_feedback(self.stage, SPEC)
        self.controller.create_graph.assert_called_once()
        self.assertEqual(self.controller.create_node.call_count, 6)
        self.assertEqual(self.controller.connect.call_count, 10)
        self.execute.assert_called_once()
        self.assertTrue(old_graph)
        imu_path = self.robot + '/imu_link/ImuSensor'
        self.assertAlmostEqual(self.stage.GetPrimAtPath(imu_path).GetAttribute('sensorPeriod').Get(), 0.01)
        read = self.stage.GetPrimAtPath(GRAPH_PATH + '/ReadIMU')
        self.assertEqual(read.GetRelationship('inputs:imuPrim').GetTargets(), [Sdf.Path(imu_path)])
        publisher = self.stage.GetPrimAtPath(GRAPH_PATH + '/PublishJoints')
        self.assertEqual(publisher.GetRelationship('inputs:targetPrim').GetTargets(),
                         [Sdf.Path(self.robot + '/base_footprint')])
        self.assertEqual(publisher.GetAttribute('inputs:topicName').Get(), '/joint_states')
        self.assertEqual(self.stage.GetAttributeAtPath(
            GRAPH_PATH + '/PublishIMU.inputs:orientation').GetConnections(),
            [Sdf.Path(GRAPH_PATH + '/ReadIMU.outputs:orientation')])

    def test_missing_wheel_is_rejected(self):
        self.stage.RemovePrim(self.robot + '/joints/front_left_wheel_joint')
        with self.assertRaisesRegex(RuntimeError, 'Missing wheel joints'):
            find_feedback_prims(self.stage, SPEC['wheels']['order'])

    def test_sensor_path_collision_is_not_overwritten(self):
        path = self.robot + '/imu_link/ImuSensor'
        self.stage.DefinePrim(path, 'Xform')
        with patch.dict(sys.modules, self.modules), self.assertRaisesRegex(RuntimeError, 'occupied'):
            create_imu_sensor(self.stage, self.robot + '/imu_link', 100)
        self.assertEqual(self.stage.GetPrimAtPath(path).GetTypeName(), 'Xform')

    def test_invalid_sensor_rate_is_rejected(self):
        for rate in (0, -1, float('nan'), float('inf')):
            with patch.dict(sys.modules, self.modules), self.assertRaises(ValueError):
                create_imu_sensor(self.stage, self.robot + '/imu_link', rate)

    def test_wheel_message_order_is_resolved_by_name(self):
        names = [name + '_wheel_joint' for name in SPEC['wheels']['order']]
        message = SimpleNamespace(name=list(reversed(names)),
                                  position=[1, 2, 3, 4], velocity=[5, 6, 7, 8])
        result = wheel_values(message, names)
        self.assertEqual(result[names[0]], {'position_rad': 4, 'velocity_rad_s': 8})
        self.assertEqual(result[names[3]], {'position_rad': 1, 'velocity_rad_s': 5})

    def test_imu_message_frame_and_numeric_validation(self):
        message = SimpleNamespace(header=SimpleNamespace(frame_id='imu_link'),
                                  angular_velocity=SimpleNamespace(x=0, y=0, z=0.2),
                                  linear_acceleration=SimpleNamespace(x=0, y=0, z=9.81))
        self.assertEqual(imu_values(message)['angular_velocity_rad_s'][2], 0.2)
        message.angular_velocity.z = float('nan')
        with self.assertRaises(ValueError):
            imu_values(message)
        message.header.frame_id = 'wrong_frame'
        with self.assertRaisesRegex(ValueError, 'Unexpected IMU frame'):
            imu_values(message)

    def test_launcher_isolates_system_ros_libraries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / 'exts/isaacsim.ros2.bridge/jazzy/lib'
            bridge.mkdir(parents=True)
            runtime = root / 'python.sh'
            runtime.write_text('#!/usr/bin/python3\nimport json, os, sys\n'
                               'print(json.dumps({"args": sys.argv[1:], "env": '
                               '{key: os.environ.get(key) for key in '
                               '("LD_LIBRARY_PATH", "ROS_DISTRO", "PYTHONPATH", "AMENT_PREFIX_PATH")}}))\n')
            runtime.chmod(0o755)
            env = dict(os.environ, ISAAC_SIM_PYTHON=str(runtime), ISAAC_SIM_ROOT=str(root),
                       LD_LIBRARY_PATH='/opt/ros/jazzy/lib', PYTHONPATH='/opt/ros/jazzy/python',
                       AMENT_PREFIX_PATH='/opt/ros/jazzy', ROS_DISTRO='humble')
            output = subprocess.check_output(['bash', str(SIM_DIR / 'run.sh'), '--vx', '0.2'],
                                             env=env, text=True)
        actual = json.loads(output)
        self.assertEqual(actual['args'], ['run_amr.py', '--vx', '0.2'])
        self.assertEqual(actual['env']['LD_LIBRARY_PATH'], str(bridge))
        self.assertEqual(actual['env']['ROS_DISTRO'], 'jazzy')
        self.assertIsNone(actual['env']['PYTHONPATH'])
        self.assertIsNone(actual['env']['AMENT_PREFIX_PATH'])


if __name__ == '__main__':
    unittest.main()
