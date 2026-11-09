#!/usr/bin/env python3
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

"""
Exercise native EKF, covariance relay and straight control with synthetic DDS sensors.

Use an isolated ROS_DOMAIN_ID without Isaac Sim or other command publishers.
This tests the pipeline and failure handling, not real contact/slip or 800 kg dynamics.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Empty, Trigger
from tf2_msgs.msg import TFMessage


class Fixture:
    """Publish measured joints/IMU and advance a deterministic planar fixture."""

    def __init__(self, node):
        """Create synthetic sensor streams and collect real controller outputs."""
        self.node = node
        self.joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.imu = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.clock = node.create_publisher(Clock, '/clock', 10)
        self.target = [0.0]*4
        self.angles = [1.0, 2.0, 3.0, 4.0]
        self.pose = [0.0, 0.0, 0.0]
        self.sim_ns = 1000000000
        self.mode = 'normal'
        self.disturb_until = 0.0
        self.commands, self.output, self.conditioned, self.transforms = [], [], [], []
        node.create_subscription(JointState, '/wheel_commands', self.receive_targets, 10)
        node.create_subscription(Twist, '/cmd_vel', self.commands.append, 100)
        node.create_subscription(Odometry, '/odom', self.output.append, 100)
        node.create_subscription(Imu, '/imu/filtered_input', self.conditioned.append,
                                 qos_profile_sensor_data)
        node.create_subscription(TFMessage, '/tf',
                                 lambda m: self.transforms.extend(m.transforms), 100)

    def receive_targets(self, message):
        """Map the real drive controller's targets by wheel name."""
        self.target = [message.velocity[message.name.index(name + '_wheel_joint')]
                       for name in ('front_left', 'front_right', 'rear_left', 'rear_right')]

    def step(self):
        """Advance sensors; optionally freeze IMU or simulation time for watchdog proof."""
        dt = 0.02
        if self.mode != 'paused':
            self.sim_ns += 20000000
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = divmod(self.sim_ns, 1000000000)
        clock = Clock()
        clock.clock = stamp
        self.clock.publish(clock)
        disturbed = self.sim_ns*1e-9 < self.disturb_until
        lateral = 0.03 if disturbed else 0.0
        actual = [v + sign*lateral/0.075
                  for v, sign in zip(self.target, (-1, 1, 1, -1))]
        fl, fr, rl, rr = actual
        vx = 0.075*(fl+fr+rl+rr)/4
        vy = 0.075*(-fl+fr+rl-rr)/4
        wz = 0.075*(-fl+fr-rl+rr)/(4*0.33) + (0.12 if disturbed else 0.0)
        if self.mode != 'paused':
            self.pose[0] += (math.cos(self.pose[2])*vx - math.sin(self.pose[2])*vy)*dt
            self.pose[1] += (math.sin(self.pose[2])*vx + math.cos(self.pose[2])*vy)*dt
            self.pose[2] += wz*dt
            self.angles = [a+v*dt for a, v in zip(self.angles, actual)]
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [n + '_wheel_joint' for n in
                       ('front_left', 'front_right', 'rear_left', 'rear_right')]
        joints.position = [(a+2*math.pi) % (4*math.pi) - 2*math.pi for a in self.angles]
        joints.velocity = actual
        if self.mode != 'wheel_dropout':
            self.joints.publish(joints)
        imu = Imu()
        imu.header.stamp = self.frozen_stamp if self.mode == 'stale_imu' else stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation.z = math.sin(self.pose[2]/2)
        imu.orientation.w = math.cos(self.pose[2]/2)
        imu.angular_velocity.z = wz
        if self.mode == 'invalid_imu':
            imu.angular_velocity.z = math.nan
        imu.linear_acceleration.z = 9.81
        self.imu.publish(imu)
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0)
        time.sleep(0.015)

    def wait(self, predicate, reason, seconds=10):
        """Pump live sensors while awaiting an observable condition."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.step()
            if predicate():
                return
        raise AssertionError(reason)

    def call(self, name):
        """Call one Trigger service without starving sensor streams."""
        client = self.node.create_client(Trigger, '/straight_drive/' + name)
        self.wait(lambda: client.service_is_ready(), 'Missing service ' + name)
        future = client.call_async(Trigger.Request())
        self.wait(future.done, 'Service timed out ' + name)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def run_for(self, seconds):
        """Continue publishing for a wall-clock observation window."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.step()


def moving(message):
    """Detect nonzero planar Twist commands."""
    return max(abs(message.linear.x), abs(message.linear.y), abs(message.angular.z)) > 1e-5


def main():
    """Prove startup, fusion, correction, TF ownership and independent sensor watchdogs."""
    root = Path(tempfile.mkdtemp(prefix='amr-straight-dds-'))
    params = root / 'straight.yaml'
    params.write_text('straight_drive:\n  ros__parameters:\n    distance_m: 0.3\n')
    log = (root / 'bringup.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'my_amr', 'bringup.launch.py', 'straight:=true',
         'straight_params:=' + str(params)], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('straight_fixture')
    fixture = Fixture(node)
    try:
        fixture.wait(lambda: len(fixture.output) > 20 and fixture.conditioned,
                     'EKF/IMU pipeline did not start', 20)
        fixture.run_for(0.5)
        assert not any(moving(m) for m in fixture.commands), 'Automatic motion before start'
        imu = fixture.conditioned[-1]
        assert imu.orientation_covariance[8] == 0.01
        assert imu.angular_velocity_covariance[8] == 0.0025
        assert imu.linear_acceleration.z == 9.81
        assert len(node.get_publishers_info_by_topic('/odom')) == 1
        assert len(node.get_publishers_info_by_topic('/odometry/wheel')) == 1
        for owner, expected in [('wheel_odometry', 'False'), ('ekf_filter', 'True')]:
            text = subprocess.check_output(
                ['ros2', 'param', 'get', '/' + owner, 'publish_tf'], text=True)
            assert expected in text, (owner, text)
        assert fixture.call('start').success
        assert not fixture.call('start').success, 'Duplicate start accepted'
        fixture.disturb_until = fixture.sim_ns*1e-9 + 1.0
        fixture.wait(lambda: 'stopped: completed' in (root/'bringup.log').read_text(),
                     'Straight trial did not converge', 25)
        assert abs(fixture.pose[0] - 0.3) < 0.025, fixture.pose
        assert abs(fixture.pose[1]) < 0.02, fixture.pose
        assert abs(fixture.pose[2]) < 0.04, fixture.pose
        assert any(m.linear.y < -0.005 for m in fixture.commands)
        assert any(m.angular.z < -0.02 for m in fixture.commands)
        assert any(t.child_frame_id == 'base_footprint' for t in fixture.transforms)
        print('PASS native EKF, IMU covariance, TF owner, signed correction, finite completion',
              fixture.pose, flush=True)
        fixture.disturb_until = 0.0

        for mode in ('stale_imu', 'invalid_imu', 'wheel_dropout', 'paused', 'time_reset',
                     'conflict'):
            fixture.mode = 'normal'
            fixture.run_for(0.5)
            response = fixture.call('start')
            assert response.success, response.message
            fixture.wait(lambda: fixture.commands and moving(fixture.commands[-1]),
                         'Trial did not move before fault')
            fixture.frozen_stamp = fixture.conditioned[-1].header.stamp
            conflicting = None
            if mode == 'conflict':
                conflicting = node.create_publisher(Twist, '/cmd_vel', 1)
            elif mode == 'time_reset':
                fixture.sim_ns = 1000000000
            else:
                fixture.mode = mode
            fixture.wait(lambda: fixture.commands and not moving(fixture.commands[-1]),
                         'Fault did not stop motion: ' + mode, 3)
            fixture.mode = 'normal'
            fixture.run_for(0.6)
            assert not moving(fixture.commands[-1]), 'Automatic restart after fault'
            if conflicting:
                assert not fixture.call('start').success
                node.destroy_publisher(conflicting)
            print('PASS', mode, 'stops; recovered sensors do not auto-resume', flush=True)
            if mode == 'time_reset':
                reset = node.create_client(Empty, '/ekf_filter/reset')
                fixture.wait(reset.service_is_ready, 'Missing native EKF reset service')
                future = reset.call_async(Empty.Request())
                fixture.wait(future.done, 'EKF reset service failed')
                assert future.exception() is None
                node.destroy_client(reset)
                fixture.run_for(0.8)
                print('PASS explicit native EKF reset after clock rewind', flush=True)
        fixture.run_for(0.5)
        assert fixture.call('start').success
        assert fixture.call('stop').success
        fixture.run_for(0.2)
        assert not moving(fixture.commands[-1])
        assert process.poll() is None
        print('PASS explicit restart and stop service', flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8)
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        print('Reports:', root, flush=True)


if __name__ == '__main__':
    main()
