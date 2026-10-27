#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check graph wiring against real USD with an offline OmniGraph adapter."""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from pxr import Sdf, Usd


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_commands import GraphTwistSubscriber


class GraphChecks(unittest.TestCase):
    def test_existing_graph_is_reused_and_receipts_reach_command_buffer(self):
        stage = Usd.Stage.CreateInMemory()
        graph = '/World/ActionGraph'
        subscriber = stage.DefinePrim(graph + '/twist', 'OmniGraphNode')
        subscriber.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(
            'isaacsim.ros2.bridge.ROS2SubscribeTwist')
        subscriber.CreateAttribute('inputs:topicName', Sdf.ValueTypeNames.String).Set('/cmd_vel')
        subscriber.CreateAttribute('outputs:execOut', Sdf.ValueTypeNames.Token)
        linear = subscriber.CreateAttribute('outputs:linearVelocity', Sdf.ValueTypeNames.Double3)
        angular = subscriber.CreateAttribute('outputs:angularVelocity', Sdf.ValueTypeNames.Double3)
        clock = stage.DefinePrim(graph + '/clock', 'OmniGraphNode')
        clock.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(
            'isaacsim.ros2.bridge.ROS2PublishClock')

        def create_node(path, node_type):
            self.assertEqual(path, graph + '/twist_received_count')
            node = stage.DefinePrim(path, 'OmniGraphNode')
            node.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(node_type)
            node.CreateAttribute('inputs:execIn', Sdf.ValueTypeNames.Token)
            node.CreateAttribute('outputs:count', Sdf.ValueTypeNames.Int).Set(0)

        def attribute(path):
            value = stage.GetAttributeAtPath(path)
            self.assertTrue(value, path)
            return SimpleNamespace(get=value.Get, usd=value)

        def connect(source, destination):
            destination.usd.AddConnection(source.usd.GetPath())

        controller = SimpleNamespace(Keys=SimpleNamespace(CREATE_NODES='create'),
                                     edit=Mock(side_effect=AssertionError('Existing graph must not be recreated')),
                                     create_node=Mock(side_effect=create_node), attribute=attribute,
                                     connect=Mock(side_effect=connect))
        modules = {name: ModuleType(name) for name in ('omni', 'omni.graph', 'omni.graph.core')}
        modules['omni'].graph = modules['omni.graph']
        modules['omni.graph'].core = modules['omni.graph.core']
        modules['omni.graph.core'].Controller = controller
        with patch.dict(sys.modules, modules), patch('ros_commands.time.monotonic', return_value=1.0):
            source = GraphTwistSubscriber(stage, 0.25)
            self.assertEqual(source.poll(), [0]*3)
            linear.Set((0.1, 0.2, 0))
            angular.Set((0, 0, 0.3))
            stage.GetAttributeAtPath(graph + '/twist_received_count.outputs:count').Set(1)
            self.assertEqual(source.poll(), [0.1, 0.2, 0.3])
            GraphTwistSubscriber(stage, 0.25)
        controller.edit.assert_not_called()
        controller.create_node.assert_called_once_with(
            graph + '/twist_received_count', 'omni.graph.action.Counter')
        controller.connect.assert_called_once()
        self.assertEqual(clock.GetAttribute('node:type').Get(),
                         'isaacsim.ros2.bridge.ROS2PublishClock')


if __name__ == '__main__':
    unittest.main()
