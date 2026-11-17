#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Exercise installed SLAM with room scans in an isolated domain, without robot commands."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

# Never share a domain with the user's robot or mapping session.
os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 20)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener


def run_case(root, topic, name):
    """Check initial mapping, lateral tracking, native saving and scan-topic overrides."""
    log_path = root / (name + '.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'amr_mapping', 'mapping.launch.py',
             'start_bringup:=false', 'rviz:=false', 'scan_topic:=' + topic],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = rclpy.create_node('mapping_fixture_' + name)
        try:
            spec = json.loads((Path(get_package_share_directory('amr_description')) /
                               'config/compact_amr.json').read_text())
            sensor = spec['sensors'][name]
            clock_pub = node.create_publisher(Clock, '/clock', 10)
            scan_pub = node.create_publisher(LaserScan, topic, 10)
            maps = []
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            map_sub = node.create_subscription(OccupancyGrid, '/map', maps.append, map_qos)
            static = StaticTransformBroadcaster(node)
            mounting = TransformStamped()
            mounting.header.frame_id = 'base_footprint'
            mounting.child_frame_id = name + '_scan'
            mounting.transform.translation.x, mounting.transform.translation.y = sensor['xyz_m'][:2]
            mounting.transform.translation.z = sensor['xyz_m'][2] + 0.025
            yaw = sensor['rpy_rad'][2]
            mounting.transform.rotation.z = math.sin(yaw / 2)
            mounting.transform.rotation.w = math.cos(yaw / 2)
            static.sendTransform(mounting)
            odom = TransformBroadcaster(node)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            state_client = node.create_client(GetState, '/slam_toolbox/get_state')
            params_client = node.create_client(GetParameters, '/slam_toolbox/get_parameters')
            simulated_ns = 1000000000

            def step(y=0.0):
                """Publish known odometry and geometrically consistent 270-degree scans."""
                nonlocal simulated_ns
                assert process.poll() is None, log_path.read_text()
                simulated_ns += 50000000
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(simulated_ns, 1000000000)
                clock_pub.publish(clock)
                transform = TransformStamped()
                transform.header.stamp = clock.clock
                transform.header.frame_id = 'odom'
                transform.child_frame_id = 'base_footprint'
                transform.transform.translation.y = y
                transform.transform.rotation.w = 1.0
                odom.sendTransform(transform)
                scan = LaserScan()
                scan.header.stamp = clock.clock
                scan.header.frame_id = mounting.child_frame_id
                scan.angle_min = math.radians(-sensor['horizontal_fov_deg'] / 2)
                scan.angle_increment = math.radians(sensor['horizontal_resolution_deg'])
                count = round(sensor['horizontal_fov_deg'] / sensor['horizontal_resolution_deg'])
                scan.angle_max = scan.angle_min + (count - 1) * scan.angle_increment
                scan.range_min, scan.range_max = sensor['range_m']
                scan.scan_time = 0.05
                ox, oy = sensor['xyz_m'][0], y + sensor['xyz_m'][1]
                for i in range(count):
                    angle = yaw + scan.angle_min + i * scan.angle_increment
                    dx, dy = math.cos(angle), math.sin(angle)
                    hits = [scan.range_max]
                    # Asymmetric bounded room; both horizontal and vertical walls.
                    for wx in (-3.0, 4.0):
                        if abs(dx) > 1e-9:
                            distance = (wx - ox) / dx
                            if distance > scan.range_min and -2.5 <= oy + distance * dy <= 3.5:
                                hits.append(distance)
                    for wy in (-2.5, 3.5):
                        if abs(dy) > 1e-9:
                            distance = (wy - oy) / dy
                            if distance > scan.range_min and -3.0 <= ox + distance * dx <= 4.0:
                                hits.append(distance)
                    scan.ranges.append(min(hits))
                scan_pub.publish(scan)
                # Drain both TF streams and map callbacks; one callback per scan
                # leaves this observer behind the simulator and distorts pose checks.
                for _ in range(6):
                    rclpy.spin_once(node, timeout_sec=0.002)
                time.sleep(0.02)

            def until(predicate, y=0.0, timeout=15):
                deadline = time.monotonic() + timeout
                while not predicate():
                    assert time.monotonic() < deadline, 'Timed out: ' + str(log_path)
                    step(y)

            until(lambda: state_client.service_is_ready() and params_client.service_is_ready())
            state = state_client.call_async(GetState.Request())
            until(state.done)
            # Configuration may still be in flight when the service first appears.
            deadline = time.monotonic() + 10
            while state.result().current_state.id != 3:
                assert time.monotonic() < deadline, 'SLAM did not auto-activate'
                step()
                state = state_client.call_async(GetState.Request())
                until(state.done)
            request = GetParameters.Request()
            request.names = ['scan_topic', 'use_sim_time']
            params = params_client.call_async(request)
            until(params.done)
            assert params.result().values[0].string_value == topic
            assert params.result().values[1].bool_value
            until(lambda: node.count_subscribers(topic) >= 1 and bool(maps))
            # A stationary initial scan must already produce meaningful map cells.
            assert any(value == 0 for value in maps[-1].data), 'No free map cells'
            initial_map_count = len(maps)
            until(lambda: buffer.can_transform('map', 'base_footprint', Time()))
            initial = buffer.lookup_transform('map', 'base_footprint', Time())
            for i in range(81):
                step(-i * 0.01)
            until(lambda: len(maps) > initial_map_count, y=-0.8)
            for _ in range(25):
                step(-0.8)
            final = buffer.lookup_transform('map', 'base_footprint', Time())
            delta_y = final.transform.translation.y - initial.transform.translation.y
            assert abs(delta_y + 0.8) < 0.2, ('Lateral pose mismatch', delta_y)
            assert any(value >= 50 for value in maps[-1].data), 'No occupied map cells'
            map_path = root / (name + '_map')
            with (root / (name + '_save.log')).open('w') as save_log:
                saver = subprocess.Popen(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(map_path),
                     '--ros-args', '-p', 'save_map_timeout:=10.0',
                     '-p', 'map_subscribe_transient_local:=true'],
                    stdout=save_log, stderr=subprocess.STDOUT)
                try:
                    until(lambda: saver.poll() is not None, y=-0.8)
                    assert saver.returncode == 0, 'Map saver failed; see ' + str(root)
                finally:
                    if saver.poll() is None:
                        saver.terminate()
                        saver.wait(timeout=5)
            for suffix in ('.yaml', '.pgm'):
                assert map_path.with_suffix(suffix).stat().st_size > 0
            graph_path = root / (name + '_graph')
            client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
            until(client.service_is_ready, y=-0.8)
            request = SerializePoseGraph.Request()
            request.filename = str(graph_path)
            future = client.call_async(request)
            until(future.done, y=-0.8)
            assert future.result().result == 0, future.result()
            for suffix in ('.posegraph', '.data'):
                assert graph_path.with_suffix(suffix).stat().st_size > 0
            node.destroy_client(client)
            print(f'PASS {topic}: active, scan parameter, initial map, lateral dy={delta_y:.3f}m, '
                  'map TF and native saving', flush=True)
        except Exception:
            print(log_path.read_text()[-6000:], flush=True)
            raise
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            node.destroy_node()


if __name__ == '__main__':
    root = Path(tempfile.mkdtemp(prefix='amr-mapping-'))
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    print('Mapping test artifacts:', root, 'domain', os.environ['ROS_DOMAIN_ID'], flush=True)
    rclpy.init()
    try:
        run_case(root, '/scan/front', 'lidar_front_left')
        run_case(root, '/scan/rear', 'lidar_rear_right')
    finally:
        rclpy.shutdown()
