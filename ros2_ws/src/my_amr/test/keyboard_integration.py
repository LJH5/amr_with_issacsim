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
Exercise real C++ keyboard/controller processes through a PTY and ROS DDS.

Run in system Jazzy after sourcing install/setup.bash. --sim also requires an
Isaac Sim instance in ROS control mode on the same isolated ROS_DOMAIN_ID.
WARNING: --sim moves the robot; use an empty test scene.
"""

import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    """Run WASD, Q/E rotation, J stop and watchdog checks against real executables."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', action='store_true')
    args = parser.parse_args()
    spec = json.loads((Path(get_package_share_directory('amr_description')) /
                       'config/compact_amr.json').read_text())
    names = [name + '_wheel_joint' for name in spec['wheels']['order']]
    radius = spec['wheels']['radius_m']
    speed = 0.1
    rclpy.init()
    node = rclpy.create_node('keyboard_integration_check')
    state = {'cmd': None, 'wheels': None, 'actual': None}

    def wheels(key, message):
        try:
            state[key] = [message.velocity[message.name.index(name)] for name in names]
        except (ValueError, IndexError):
            pass

    def receive_cmd(message):
        state['cmd'] = [message.linear.x, message.linear.y, message.angular.z]

    node.create_subscription(Twist, spec['control']['cmd_vel_topic'], receive_cmd, 1)
    node.create_subscription(JointState, spec['control']['wheel_command_topic'],
                             lambda msg: wheels('wheels', msg), 1)
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: wheels('actual', msg), qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, spec['control']['cmd_vel_topic'], 1)

    def spin_for(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait_for(predicate, description, seconds=5):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'{description}: {state}')

    def matched(expected):
        command_ok = state['wheels'] is not None and all(
            abs(a - b) < 0.05 for a, b in zip(state['wheels'], expected))
        actual_ok = state['actual'] is not None and all(
            abs(a) < 0.15 if abs(b) < 1e-8 else a * b > 0.05
            for a, b in zip(state['actual'], expected))
        return command_ok and (not args.sim or actual_ok)

    master, slave = pty.openpty()
    executable = Path(get_package_prefix('my_amr')) / 'lib/my_amr/keyboard_teleop'
    with tempfile.TemporaryDirectory(prefix='amr_keyboard_test_') as directory:
        log = open(Path(directory) / 'process.log', 'w+')
        controller = subprocess.Popen(
            ['ros2', 'launch', 'my_amr', 'drive.launch.py'], stdout=log,
            stderr=log, start_new_session=True)
        keyboard = None
        try:
            wait_for(lambda: state['wheels'] is not None, 'C++ controller startup', 15)
            if args.sim:
                wait_for(lambda: state['actual'] is not None, 'Isaac Sim startup', 90)
            keyboard = subprocess.Popen(
                [str(executable), '--ros-args', '-p', f'speed_m_s:={speed}'],
                stdin=slave, stdout=log, stderr=log)
            wait_for(lambda: state['cmd'] is not None, 'Keyboard startup')
            turn = 0.3 * (spec['wheels']['wheelbase_m'] + spec['wheels']['track_m']) / (2 * radius)
            commands = [(key, [sign * speed / radius for sign in signs])
                        for key, signs in (('w', (1, 1, 1, 1)), ('s', (-1, -1, -1, -1)),
                                           ('a', (-1, 1, 1, -1)), ('d', (1, -1, -1, 1)))]
            commands.extend([('q', [-turn, turn, -turn, turn]),
                             ('e', [turn, -turn, turn, -turn])])
            for key, expected in commands:
                os.write(master, key.encode())
                wait_for(lambda: matched(expected), 'Key ' + key)
                # Same-valued repeated messages must remain fresh beyond timeout.
                spin_for(0.4)
                assert matched(expected), f'Hold {key}: {state}'
                print(f'PASS {key}: command={state["wheels"]}, actual={state["actual"]}',
                      flush=True)
                os.write(master, b'J')
                wait_for(lambda: matched([0] * 4), 'J stop')
                assert keyboard.poll() is None, 'J must stop without exiting'
            os.write(master, b'w')
            wait_for(lambda: matched([speed / radius] * 4), 'Watchdog precondition')
            keyboard.kill()
            keyboard.wait(timeout=5)
            wait_for(lambda: matched([0] * 4), 'cmd_vel timeout after keyboard death')
            print('PASS J stop and C++ cmd_vel watchdog', flush=True)
            if args.sim:
                command = Twist()
                command.linear.x = speed
                until = time.monotonic() + 1.0
                while time.monotonic() < until:
                    publisher.publish(command)
                    spin_for(0.05)
                assert matched([speed / radius] * 4), 'Transport watchdog precondition'
                # Kill the controller while a nonzero wheel command is active.
                os.killpg(controller.pid, signal.SIGTERM)
                controller.wait(timeout=5)
                wait_for(lambda: all(abs(v) < 0.15 for v in state['actual']),
                         'Simulator transport watchdog after controller death')
                print('PASS Isaac wheel-command transport watchdog', flush=True)
            else:
                publisher.publish(Twist())
            return 0
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            if keyboard is not None and keyboard.poll() is None:
                keyboard.send_signal(signal.SIGINT)
                keyboard.wait(timeout=5)
            if controller.poll() is None:
                os.killpg(controller.pid, signal.SIGTERM)
                controller.wait(timeout=5)
            os.close(master)
            os.close(slave)
            log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
