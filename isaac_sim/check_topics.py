#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check IMU, wheel feedback, both LiDAR scans and sensor TF using ROS 2 Jazzy."""

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


def scan_values(message, name, config):
    if message.header.frame_id != name + '_scan':
        raise ValueError('Unexpected scan frame: ' + message.header.frame_id)
    metadata = (message.angle_min, message.angle_max, message.angle_increment,
                message.range_min, message.range_max, message.scan_time, message.time_increment)
    if not all(math.isfinite(value) for value in metadata):
        raise ValueError(f'Non-finite LaserScan metadata: {metadata}')
    count = round(config['horizontal_fov_deg'] / config['horizontal_resolution_deg'])
    if len(message.ranges) != count or message.angle_increment <= 0:
        raise ValueError(f'Expected {count} horizontal beams, got {len(message.ranges)}')
    if not math.isclose(message.angle_increment,
                        math.radians(config['horizontal_resolution_deg']), abs_tol=1e-5):
        raise ValueError('Unexpected angular resolution')
    if not math.isclose(message.angle_min, -math.radians(config['horizontal_fov_deg']) / 2,
                        abs_tol=0.02):
        raise ValueError('Scan is not centered on local +X')
    if not math.isclose(message.angle_max,
                        message.angle_min + (count - 1) * message.angle_increment, abs_tol=0.02):
        raise ValueError('Inconsistent LaserScan angle bounds')
    if not math.isclose(message.scan_time, 1 / config['rate_hz'], rel_tol=1e-4):
        raise ValueError('Unexpected scan duration')
    if message.time_increment < 0:
        raise ValueError('Negative beam time increment')
    if any(math.isnan(value) or value < 0 for value in message.ranges):
        raise ValueError('Invalid scan range')
    if not all(math.isclose(actual, expected, rel_tol=1e-5)
               for actual, expected in zip((message.range_min, message.range_max), config['range_m'])):
        raise ValueError('Unexpected range limits')
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    if stamp <= 0:
        raise ValueError('Missing simulation timestamp')
    center = round(-message.angle_min / message.angle_increment)
    hits = [value for value in message.ranges
            if math.isfinite(value) and message.range_min <= value < message.range_max]
    return {'frame': message.header.frame_id, 'beams': count, 'stamp_s': stamp,
            'scan_time_s': message.scan_time, 'hits': len(hits),
            'nearest_m': min(hits) if hits else None,
            'forward_m': message.ranges[center]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--require-motion', action='store_true',
                        help='Also require a measured wheel speed above 0.05 rad/s')
    parser.add_argument('--fixture', action='store_true',
                        help='Require the moving 1.1 -> 1.6 m targets from test/run_lidar_fixture.py')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    from lidar import LIDAR_NAMES, SCAN_HEIGHT_M

    spec = json.loads((Path(__file__).resolve().parent.parent /
                       'ros2_ws/src/amr_description/config/compact_amr.json').read_text())
    names = [name + '_wheel_joint' for name in spec['wheels']['order']]
    samples, feedback, errors, frames = {}, {}, {}, set()
    phases = {name: set() for name in LIDAR_NAMES}
    stamps = {name: set() for name in LIDAR_NAMES}

    def receive_feedback(key, extract, message):
        try:
            feedback[key] = extract(message)
        except (ValueError, IndexError) as error:
            errors[key] = str(error)

    def receive(name, message):
        try:
            value = scan_values(message, name, spec['sensors'][name])
            samples[name] = value
            stamps[name].add(value['stamp_s'])
            for phase, distance in (('near', 1.1), ('far', 1.6)):
                if abs(value['forward_m'] - distance) < 0.08:
                    phases[name].add(phase)
        except (ValueError, IndexError) as error:
            errors[name] = str(error)

    def receive_tf(message):
        for transform in message.transforms:
            for name in LIDAR_NAMES:
                if transform.child_frame_id != name + '_scan':
                    continue
                config = spec['sensors'][name]
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                expected = list(config['xyz_m'])
                expected[2] += SCAN_HEIGHT_M - spec['chassis']['center_height_m']
                actual = (translation.x, translation.y, translation.z)
                yaw = config['rpy_rad'][2]
                quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
                expected_quaternion = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
                position_ok = all(math.isclose(a, b, abs_tol=1e-4) for a, b in zip(actual, expected))
                rotation_ok = abs(sum(a * b for a, b in zip(quaternion, expected_quaternion))) > 0.99999
                if transform.header.frame_id == 'base_link' and position_ok and rotation_ok:
                    frames.add(transform.child_frame_id)
                else:
                    errors[name + '_tf'] = 'Unexpected scan mounting transform'

    def ready():
        feedback_ready = 'imu' in feedback and 'wheels' in feedback
        moving = feedback_ready and any(
            abs(item['velocity_rad_s']) > 0.05 for item in feedback['wheels'].values())
        return feedback_ready and (not args.require_motion or moving) and all(
            name in samples and len(stamps[name]) >= 3 and name + '_scan' in frames
            and (not args.fixture or phases[name] == {'near', 'far'}) for name in LIDAR_NAMES)

    rclpy.init()
    node = rclpy.create_node('amr_topics_check')
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: receive_feedback(
                                 'wheels', lambda item: wheel_values(item, names), msg),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, spec['sensors']['imu']['topic'],
                             lambda msg: receive_feedback('imu', imu_values, msg),
                             qos_profile_sensor_data)
    for name in LIDAR_NAMES:
        node.create_subscription(LaserScan, spec['sensors'][name]['topic'],
                                 lambda msg, key=name: receive(key, msg), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', receive_tf,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < deadline and not ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        rates = {name: (len(values) - 1) / (max(values) - min(values))
                 for name, values in stamps.items() if len(values) > 1}
        print(json.dumps({'feedback': feedback, 'scans': samples,
                          'simulation_rate_hz': rates, 'static_frames': sorted(frames),
                          'fixture_phases': {key: sorted(value) for key, value in phases.items()},
                          'errors': errors}, indent=2))
        print('PASS: IMU, wheels, both LiDAR scans and static TF received' if ready() and not errors
              else 'FAIL: incomplete or invalid AMR topic data')
        return 0 if ready() and not errors else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
