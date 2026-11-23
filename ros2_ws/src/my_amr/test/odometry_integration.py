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
Exercise the real C++ odometry and robot TF with synthetic measured joints.

Run after install/setup.bash in an isolated ROS_DOMAIN_ID without Isaac Sim.
This checks the ROS pipeline, not physical accuracy or friction.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


NAMES = [name + '_wheel_joint' for name in
         ('front_left', 'front_right', 'rear_left', 'rear_right')]


def run_case(publish_tf, wrapped=False, bringup=False):
    """Check names, timestamps, body twist, reset/gap rejection and TF ownership."""
    node = rclpy.create_node('odometry_integration_check')
    output, transforms = [], []
    node.create_subscription(Odometry, '/odom', output.append, 100)
    node.create_subscription(TFMessage, '/tf',
                             lambda m: transforms.extend(m.transforms), 100)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    clock = node.create_publisher(Clock, '/clock', 1)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait(predicate, description, seconds=8):
        until = time.monotonic() + seconds
        while not predicate():
            assert time.monotonic() < until, description
            rclpy.spin_once(node, timeout_sec=0.01)

    def message(stamp_ns, positions):
        m = JointState()
        m.header.stamp.sec = stamp_ns // 1000000000
        m.header.stamp.nanosec = stamp_ns % 1000000000
        # Include an unrelated joint and reverse the wheel order. Velocity is
        # deliberately absent: odometry must use measured position differences.
        m.name = ['unrelated_joint'] + list(reversed(NAMES))
        measured = [math.remainder(v, 4 * math.pi) for v in positions] if wrapped else positions
        m.position = [99.0] + list(reversed(measured))
        return m

    def send(m):
        c = Clock()
        c.clock = m.header.stamp
        clock.publish(c)
        publisher.publish(m)

    def deliver(stamp_ns, positions):
        m = message(stamp_ns, positions)
        send(m)
        wait(lambda: output and output[-1].header.stamp == m.header.stamp, 'No odom for sample')
        spin(0.05)
        return output[-1]

    def near(actual, expected):
        assert math.isclose(actual, expected, abs_tol=1e-8), (actual, expected)

    with tempfile.TemporaryDirectory(prefix='amr_odom_test_') as folder:
        parameters = Path(folder) / 'odometry.yaml'
        parameters.write_text('wheel_odometry:\n  ros__parameters:\n'
                              '    position_wrap_period_rad: ' +
                              str(4 * math.pi if wrapped else 0.0) + '\n')
        log = (Path(folder) / 'launch.log').open('w+')
        process = subprocess.Popen(
            ['ros2', 'launch', 'my_amr',
             'bringup.launch.py' if bringup else 'odometry.launch.py',
             'publish_tf:=' + str(publish_tf).lower(), 'params_file:=' + str(parameters)],
            stdout=log, stderr=log,
            start_new_session=True)
        try:
            wait(lambda: publisher.get_subscription_count() >= 2, 'Odom/RSP discovery')
            if bringup:
                wait(lambda: all(node.get_node_names().count(name) == 1 for name in
                                 ('my_amr_node', 'wheel_odometry', 'robot_state_publisher')),
                     'Bringup must start one controller, one odometry and one RSP')
            spin(0.3)
            positions = [2 * math.pi - 0.05]*4 if wrapped else [10.0, 20.0, 30.0, 40.0]
            stamp = 1000000000
            first = deliver(stamp, positions)
            near(first.pose.pose.position.x, 0)
            near(first.pose.pose.orientation.w, 1)
            for change, expected in [([1.0]*4, (0.075, 0.0)),
                                     ([-1.0, 1.0, 1.0, -1.0], (0.075, 0.075))]:
                positions = [a + b for a, b in zip(positions, change)]
                stamp += 100000000
                m = deliver(stamp, positions)
                near(m.pose.pose.position.x, expected[0])
                near(m.pose.pose.position.y, expected[1])
            near(m.twist.twist.linear.y, 0.75)
            near(m.twist.twist.linear.x, 0)
            positions = [a + b for a, b in zip(positions, [-0.88, 0.88, -0.88, 0.88])]
            stamp += 100000000
            m = deliver(stamp, positions)
            near(2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w), 0.2)
            near(m.twist.twist.angular.z, 2.0)
            positions = [a + 1 for a in positions]
            stamp += 100000000
            m = deliver(stamp, positions)
            near(m.pose.pose.position.x, 0.075 + 0.075 * math.cos(0.2))
            near(m.pose.pose.position.y, 0.075 + 0.075 * math.sin(0.2))
            near(m.twist.twist.linear.x, 0.75)  # Body frame, not world frame.
            near(m.twist.twist.linear.y, 0)
            assert m.header.frame_id == 'odom' and m.child_frame_id == 'base_footprint'
            assert all(m.pose.covariance[k] > 0 for k in (0, 7, 14, 21, 28, 35))
            if publish_tf:
                sample_time = Time.from_msg(m.header.stamp)
                wait(lambda: buffer.can_transform('odom', 'imu_link', sample_time),
                     'Disconnected odom -> base_footprint -> base_link -> IMU')
                tf = buffer.lookup_transform('odom', 'base_footprint', sample_time)
                near(tf.transform.translation.x, m.pose.pose.position.x)
                near(tf.transform.rotation.z, m.pose.pose.orientation.z)
                assert tf.header.stamp == m.header.stamp
                mounting = buffer.lookup_transform('base_footprint', 'base_link', Time())
                near(mounting.transform.translation.z, 0.19)
            else:
                assert not any(t.child_frame_id == 'base_footprint' for t in transforms)
            before = len(output)
            bad = message(stamp, [a + 100 for a in positions])  # Duplicate time.
            send(bad)
            for mode in ('missing', 'duplicate', 'short', 'nan'):
                bad = message(stamp + 50000000, positions)
                if mode == 'missing':
                    bad.name[-1] = 'not_a_wheel'
                elif mode == 'duplicate':
                    bad.name[-1] = bad.name[-2]
                elif mode == 'short':
                    bad.position.pop()
                else:
                    bad.position[-1] = math.nan
                send(bad)
            spin(0.25)
            assert len(output) == before, 'Invalid input or silence generated new odometry'
            # Restarted simulation time and arbitrary initial encoder angles.
            stamp = 100000000
            positions = [200.0]*4
            m = deliver(stamp, positions)
            near(m.pose.pose.position.x, 0)
            near(m.pose.pose.orientation.w, 1)
            # A long gap must not integrate unobserved travel or publish a fake zero.
            stamp += 1000000000
            positions = [a + 6 for a in positions]
            before = len(output)
            send(message(stamp, positions))
            spin(0.15)
            assert len(output) == before
            positions = [a + 1 for a in positions]
            stamp += 100000000
            m = deliver(stamp, positions)
            near(m.pose.pose.position.x, 0.075)
            # Implausible encoder reset also rebases without a pose jump.
            stamp += 100000000
            positions = [a + 1000 for a in positions]
            before = len(output)
            send(message(stamp, positions))
            spin(0.15)
            assert len(output) == before
            positions = [a + 1 for a in positions]
            stamp += 100000000
            m = deliver(stamp, positions)
            near(m.pose.pose.position.x, 0.15)
            if wrapped:
                # At the configured 35 rad/s limit, 0.2 s could cover more than
                # half the wrap period. Its direction cannot be inferred safely.
                stamp += 200000000
                positions = [a + 1 for a in positions]
                before = len(output)
                send(message(stamp, positions))
                spin(0.15)
                assert len(output) == before, 'Ambiguous wrapped travel was integrated'
                stamp += 100000000
                positions = [a + 1 for a in positions]
                m = deliver(stamp, positions)
                near(m.pose.pose.position.x, 0.225)
            print('PASS measured odometry, body twist, stamps, TF=' + str(publish_tf) +
                  ', wrapped=' + str(wrapped) +
                  ', malformed input, clock reset, dropout and encoder reset', flush=True)
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            listener.unregister()
            node.destroy_node()
            log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bringup', action='store_true', help='Exercise the combined launch')
    args = parser.parse_args()
    rclpy.init()
    try:
        run_case(True, bringup=args.bringup)
        run_case(False, bringup=args.bringup)
        run_case(True, wrapped=True, bringup=args.bringup)
    finally:
        rclpy.shutdown()
