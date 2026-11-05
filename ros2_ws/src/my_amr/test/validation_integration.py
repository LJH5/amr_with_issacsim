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
Exercise the real C++ validator with synthetic, synchronized ROS feedback.

Use an isolated ROS_DOMAIN_ID with no simulator. This is a measurement-pipeline
check, not a physics/friction test. Reports remain in a temporary directory.
"""
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


NAMES = [name + '_wheel_joint' for name in
         ('front_left', 'front_right', 'rear_left', 'rear_right')]


def run_case(root, mode):
    """Run six directions or interrupt feedback while the C++ test is moving."""
    node = rclpy.create_node('validation_fixture_' + mode)
    joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    truth = node.create_publisher(PoseStamped, '/ground_truth/pose', qos_profile_sensor_data)
    conflicting = (node.create_publisher(Twist, '/cmd_vel', 1)
                   if mode == 'conflict' else None)
    target = [0.0] * 4
    moved = False
    dropped_at = None
    reset_done = False
    repetitions = 3 if mode == 'complete' else 1

    def receive(message):
        nonlocal target, moved
        target = [message.velocity[message.name.index(name)] for name in NAMES]
        moved = moved or any(abs(v) > 0.05 for v in target)

    node.create_subscription(JointState, '/wheel_commands', receive, 1)
    prefix = root / mode
    log = (root / (mode + '.log')).open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'my_amr', 'validation.launch.py', 'repetitions:=' + str(repetitions),
         'drive_seconds:=' + ('0.4' if mode == 'complete' else '3.0'),
         'settle_seconds:=0.8', 'speed_m_s:=0.1', 'output_prefix:=' + str(prefix)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    angles = [1.0, 2.0, 3.0, 4.0]
    x, y, yaw = 0.4, -0.2, 0.7
    sim_ns = 1000000000
    started = time.monotonic()
    step = 16666667
    try:
        while process.poll() is None and time.monotonic() - started < 25:
            rclpy.spin_once(node, timeout_sec=0.01)
            if conflicting is not None:
                conflicting.publish(Twist())
            dt = step * 1e-9
            fl, fr, rl, rr = target
            vx = 0.075 * (fl + fr + rl + rr) / 4
            vy = 0.075 * (-fl + fr + rl - rr) / 4
            wz = 0.075 * (-fl + fr - rl + rr) / (4 * 0.33)
            # Synthetic slip: body motion is 90% of encoder prediction.
            mid = yaw + 0.9 * wz * dt / 2
            x += 0.9 * (math.cos(mid)*vx - math.sin(mid)*vy) * dt
            y += 0.9 * (math.sin(mid)*vx + math.cos(mid)*vy) * dt
            yaw += 0.9 * wz * dt
            angles = [a + v*dt for a, v in zip(angles, target)]
            sim_ns += step
            if mode == 'time_reset' and moved and not reset_done:
                sim_ns = 100000000
                reset_done = True
            message = JointState()
            message.header.stamp.sec = sim_ns // 1000000000
            message.header.stamp.nanosec = sim_ns % 1000000000
            message.name = list(reversed(NAMES))
            message.position = list(reversed(angles))
            message.velocity = list(reversed(target))
            joints.publish(message)
            pose = PoseStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            if mode == 'disconnect' and moved:
                dropped_at = dropped_at or time.monotonic()
            if dropped_at is None:
                truth.publish(pose)
        assert process.poll() is not None, 'Validation launch did not terminate'
        status = Path(str(prefix) + '_status.txt').read_text()
        if mode == 'complete':
            assert 'completed:' in status, status
            rows = list(csv.DictReader(Path(str(prefix) + '_summary.csv').open()))
            expected = ['forward', 'backward', 'left', 'right', 'ccw', 'cw'] * repetitions
            assert [r['trial'] for r in rows] == expected
            assert [int(r['repetition']) for r in rows] == [1]*6 + [2]*6 + [3]*6
            for row in rows:
                assert row['settled'] == '1', row
                assert int(row['samples']) > 10
                for axis in ('x_m', 'y_m', 'yaw_rad'):
                    predicted = float(row['wheel_' + axis])
                    actual = float(row['truth_' + axis])
                    assert abs(actual - 0.9 * predicted) < 0.002, (axis, actual, predicted)
            assert max(float(r['position_error_m']) for r in rows) > 0.001
            samples = list(csv.DictReader(Path(str(prefix) + '_samples.csv').open()))
            assert samples and all(None not in row for row in samples)
            assert abs(float(samples[0]['world_x_m'])) > 0.1
            print('PASS 18 trials: six directions x3, initial heading, encoder order, slip CSV')
        else:
            expected_error = {'disconnect': 'timeout', 'time_reset': 'time reset',
                              'conflict': 'Multiple command publishers'}[mode]
            assert 'aborted:' in status and expected_error in status, status
            assert moved or mode == 'conflict'
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.01)
            if conflicting is not None:
                conflicting.publish(Twist())
            assert max(abs(v) for v in target) < 1e-8, target
            print('PASS', mode, 'aborts and sends zero wheel targets')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()


def check_startup_wait(root):
    """Wait safely for a late controller, then stop cleanly on interruption."""
    spec_path = Path(__file__).resolve().parents[2] / 'amr_description/config/compact_amr.json'
    wheels = json.loads(spec_path.read_text())['wheels']
    parameters = {'wheel_radius_m': wheels['radius_m'], 'wheelbase_m': wheels['wheelbase_m'],
                  'track_m': wheels['track_m'], 'roller_handedness': wheels['roller_handedness'],
                  'wheel_joint_names': NAMES, 'output_prefix': str(root / 'startup_wait')}
    command = ['ros2', 'run', 'my_amr', 'drive_validation', '--ros-args']
    for name, value in parameters.items():
        command += ['-p', name + ':=' + json.dumps(value)]
    node = rclpy.create_node('validation_startup_fixture')
    joints = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    truth = node.create_publisher(PoseStamped, '/ground_truth/pose', qos_profile_sensor_data)
    seen = []
    node.create_subscription(Twist, '/cmd_vel', seen.append, 1)
    with (root / 'startup_wait.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        target = None
        started = time.monotonic()
        count = 0
        try:
            while time.monotonic() - started < 5:
                rclpy.spin_once(node, timeout_sec=0.01)
                message = JointState()
                count += 1
                message.header.stamp.sec = 1 + count // 60
                message.header.stamp.nanosec = (count % 60) * 16000000
                message.name = NAMES
                message.position = [0.0] * 4
                message.velocity = [0.0] * 4
                joints.publish(message)
                pose = PoseStamped()
                pose.header = message.header
                pose.header.frame_id = 'world'
                pose.pose.orientation.w = 1.0
                truth.publish(pose)
                assert process.poll() is None, 'Validator failed before target stream startup'
                if time.monotonic() - started < 2:
                    assert all(m.linear.x == 0 and m.linear.y == 0 and m.angular.z == 0
                               for m in seen), 'Motion before controller discovery'
                else:
                    if target is None:
                        target = node.create_publisher(JointState, '/wheel_commands', 1)
                    target.publish(message)
                    if any(abs(m.linear.x) > 0 for m in seen):
                        break
            else:
                raise AssertionError('Validator never started after target discovery')
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            for _ in range(30):
                rclpy.spin_once(node, timeout_sec=0.01)
            status = (root / 'startup_wait_status.txt').read_text()
            assert 'aborted: interrupted' in status, status
            assert seen and seen[-1].linear.x == 0
            print('PASS late target startup waits at zero; Ctrl+C stops and marks incomplete')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5)
            node.destroy_node()


def main():
    """Run deterministic DDS checks and preserve logs for review."""
    root = Path(tempfile.mkdtemp(prefix='amr-validation-dds-'))
    rclpy.init()
    try:
        check_startup_wait(root)
        run_case(root, 'complete')
        run_case(root, 'disconnect')
        run_case(root, 'time_reset')
        run_case(root, 'conflict')
    finally:
        rclpy.shutdown()
        print('Reports:', root)


if __name__ == '__main__':
    main()
