# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Read graph Twist messages with a receipt-based, wall-clock watchdog."""

import math
import time


class TimedTwist:
    """A repeated value is fresh only when the message counter advances."""

    def __init__(self, timeout, initial_count=0):
        self.timeout = timeout
        self.reset(initial_count)

    def reset(self, count):
        self.count = count
        self.received_at = None
        self.command = [0.0, 0.0, 0.0]

    def read(self, count, linear, angular, now):
        if count < self.count:
            self.reset(count)
        elif count > self.count:
            self.count = count
            try:
                command = [float(linear[0]), float(linear[1]), float(angular[2])]
                valid = all(math.isfinite(value) for value in command)
            except (TypeError, ValueError, IndexError, OverflowError):
                valid = False
            if valid:
                self.command = command
                self.received_at = now
            else:
                self.received_at = None
                self.command = [0.0, 0.0, 0.0]
        if self.received_at is None or now - self.received_at >= self.timeout:
            return [0.0, 0.0, 0.0]
        return list(self.command)


class GraphTwistSubscriber:
    """Reuse the saved subscriber; add an Exec Out counter without saving USD."""

    def __init__(self, stage, timeout):
        import omni.graph.core as og

        subscribers = [prim for prim in stage.Traverse()
                       if prim.GetAttribute('node:type').Get()
                       == 'isaacsim.ros2.bridge.ROS2SubscribeTwist']
        if len(subscribers) != 1:
            raise RuntimeError('ROS control requires exactly one ROS2 Subscribe Twist node '
                               f'in the saved scene; found {len(subscribers)}')
        subscriber = subscribers[0]
        path = str(subscriber.GetPath())
        parent = str(subscriber.GetParent().GetPath())
        counter_name = subscriber.GetName() + '_received_count'
        counter_path = parent + '/' + counter_name
        counter = stage.GetPrimAtPath(counter_path)
        if counter:
            if counter.GetAttribute('node:type').Get() != 'omni.graph.action.Counter':
                raise RuntimeError(f'Counter path is occupied by another prim: {counter_path}')
        else:
            # Controller.edit(dict, ...) always creates a graph in this SDK.
            # The absolute node path resolves the existing saved graph instead.
            og.Controller.create_node(counter_path, 'omni.graph.action.Counter')
        source = og.Controller.attribute(path + '.outputs:execOut')
        destination = og.Controller.attribute(counter_path + '.inputs:execIn')
        # Reuse the connection if the user previously saved the runtime Counter.
        connection = path + '.outputs:execOut'
        if connection not in [str(item) for item in
                              stage.GetPrimAtPath(counter_path).GetAttribute('inputs:execIn').GetConnections()]:
            og.Controller.connect(source, destination)
        self.counter = og.Controller.attribute(counter_path + '.outputs:count')
        self.linear = og.Controller.attribute(path + '.outputs:linearVelocity')
        self.angular = og.Controller.attribute(path + '.outputs:angularVelocity')
        self.buffer = TimedTwist(timeout, int(self.counter.get() or 0))
        topic = subscriber.GetAttribute('inputs:topicName').Get() or 'cmd_vel'
        namespace = subscriber.GetAttribute('inputs:nodeNamespace').Get() or ''
        if topic != '/cmd_vel' and not (topic == 'cmd_vel' and not namespace.strip('/')):
            raise RuntimeError('Set the Twist subscriber topic to /cmd_vel with an empty namespace')
        print(f'ROS control: {path}, /cmd_vel, timeout {timeout:.2f}s')

    def poll(self):
        count = int(self.counter.get() or 0)
        return self.buffer.read(count, self.linear.get(), self.angular.get(), time.monotonic())

    def reset(self):
        self.buffer.reset(int(self.counter.get() or 0))
