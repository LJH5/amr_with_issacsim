# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Thin wheel-command transport adapter; body control lives in ROS C++."""

import math
import time

GRAPH_PATH = '/World/AMRWheelCommands'


class WheelCommandBuffer:
    """Only a new receipt counter refreshes the transport watchdog."""

    def __init__(self, names, timeout, max_speed, initial_count=0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Wheel command timeout must be positive and finite')
        self.names, self.timeout, self.max_speed = list(names), timeout, max_speed
        self.reset(initial_count)

    def reset(self, count):
        self.count, self.received_at = count, None
        self.command = [0.0] * 4

    def read(self, count, names, velocities, now):
        if count < self.count:
            self.reset(count)
        elif count > self.count:
            self.count = count
            try:
                names, velocities = list(names), list(velocities)
                if len(names) != 4 or len(velocities) != 4 or set(names) != set(self.names):
                    raise ValueError('Exactly the four named wheel velocities are required')
                command = [float(velocities[names.index(name)]) for name in self.names]
                if not all(math.isfinite(value) and abs(value) <= self.max_speed + 1e-6
                           for value in command):
                    raise ValueError('Invalid wheel velocities')
                self.command, self.received_at = command, now
            except (TypeError, ValueError, IndexError, OverflowError):
                self.received_at, self.command = None, [0.0] * 4
        if self.received_at is None or now - self.received_at >= self.timeout:
            return [0.0] * 4
        return list(self.command)


class GraphWheelSubscriber:
    """Receive JointState wheel velocities via Isaac Sim's native ROS bridge."""

    def __init__(self, stage, spec):
        import omni.graph.core as og

        graph = stage.GetPrimAtPath(GRAPH_PATH)
        if graph and graph.GetTypeName() != 'OmniGraph':
            raise RuntimeError(f'Wheel command graph path is occupied: {GRAPH_PATH}')
        if not graph:
            og.Controller.create_graph({'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'})
        types = {'Tick': 'omni.graph.action.OnPlaybackTick',
                 'Context': 'isaacsim.ros2.bridge.ROS2Context',
                 'Subscribe': 'isaacsim.ros2.bridge.ROS2SubscribeJointState',
                 'Received': 'omni.graph.action.Counter'}
        for name, node_type in types.items():
            path = GRAPH_PATH + '/' + name
            node = stage.GetPrimAtPath(path)
            if node:
                if node.GetAttribute('node:type').Get() != node_type:
                    raise RuntimeError(f'Unexpected wheel command node type: {path}')
            else:
                og.Controller.create_node(path, node_type)
        for source, destination in (
                ('Tick.outputs:tick', 'Subscribe.inputs:execIn'),
                ('Context.outputs:context', 'Subscribe.inputs:context'),
                ('Subscribe.outputs:execOut', 'Received.inputs:execIn')):
            source, destination = (GRAPH_PATH + '/' + value for value in (source, destination))
            attribute = stage.GetAttributeAtPath(destination)
            existing = [str(value) for value in attribute.GetConnections()]
            if existing and existing != [source]:
                raise RuntimeError(f'Wheel input already connected elsewhere: {destination}')
            if not existing:
                og.Controller.connect(og.Controller.attribute(source), og.Controller.attribute(destination))
        for attribute, value in {'Context.inputs:useDomainIDEnvVar': True,
                                 'Subscribe.inputs:topicName': spec['control']['wheel_command_topic'],
                                 'Subscribe.inputs:queueSize': 1}.items():
            og.Controller.set(og.Controller.attribute(GRAPH_PATH + '/' + attribute), value)
        self.counter = og.Controller.attribute(GRAPH_PATH + '/Received.outputs:count')
        self.names = og.Controller.attribute(GRAPH_PATH + '/Subscribe.outputs:jointNames')
        self.velocities = og.Controller.attribute(GRAPH_PATH + '/Subscribe.outputs:velocityCommand')
        self.buffer = WheelCommandBuffer(
            [name + '_wheel_joint' for name in spec['wheels']['order']],
            spec['motion_targets']['command_timeout_s'], spec['wheels']['joint_velocity_limit_rad_s'],
            int(self.counter.get() or 0))
        print(f'ROS wheel control: {spec["control"]["wheel_command_topic"]}, rad/s; no Twist/IK in simulator')

    def poll(self):
        return self.buffer.read(int(self.counter.get() or 0), self.names.get(),
                                self.velocities.get(), time.monotonic())

    def reset(self):
        self.buffer.reset(int(self.counter.get() or 0))
