#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check measured feedback from a separate, system ROS 2 Jazzy terminal."""

import argparse
import json
import math
from pathlib import Path
import time


def wheel_values(message, names):
    result = {}
    for name in names:
        index = message.name.index(name)
        position, velocity = message.position[index], message.velocity[index]
        if not all(math.isfinite(value) for value in (position, velocity)):
            raise ValueError(f'Non-finite wheel feedback: {name}')
        result[name] = {'position_rad': position, 'velocity_rad_s': velocity}
    return result


def imu_values(message):
    if message.header.frame_id != 'imu_link':
        raise ValueError(f'Unexpected IMU frame: {message.header.frame_id}')
    result = {'angular_velocity_rad_s': [message.angular_velocity.x,
                                       message.angular_velocity.y, message.angular_velocity.z],
              'linear_acceleration_m_s2': [message.linear_acceleration.x,
                                          message.linear_acceleration.y, message.linear_acceleration.z]}
    if not all(math.isfinite(value) for vector in result.values() for value in vector):
        raise ValueError('Non-finite IMU feedback')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--require-motion', action='store_true',
                        help='Also require a measured wheel speed above 0.05 rad/s')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState

    spec_path = Path(__file__).resolve().parent.parent / 'ros2_ws/src/amr_description/config/compact_amr.json'
    spec = json.loads(spec_path.read_text())
    names = [name + '_wheel_joint' for name in spec['wheels']['order']]
    samples, errors = {}, {}

    def receive(key, extract, message):
        try:
            samples[key] = extract(message)
        except (ValueError, IndexError) as error:
            errors[key] = str(error)

    rclpy.init()
    node = rclpy.create_node('amr_feedback_check')
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: receive('wheels', lambda item: wheel_values(item, names), msg),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, spec['sensors']['imu']['topic'],
                             lambda msg: receive('imu', imu_values, msg), qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    def ready():
        return len(samples) == 2 and (not args.require_motion or any(
            abs(item['velocity_rad_s']) > 0.05 for item in samples['wheels'].values()))

    try:
        while rclpy.ok() and time.monotonic() < deadline and not ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(samples, indent=2))
        if not ready():
            print(f'FAIL: missing feedback {sorted(set(("wheels", "imu")) - samples.keys())}; {errors}')
            return 1
        print('PASS: received measured wheel positions/velocities and IMU data')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
