#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Native Nav2 + AMCL + C++ drive/odom with synthetic hardware in a private domain."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = str(160 + os.getpid() % 20)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
import yaml


class Hardware:
    """Raycast the saved map and integrate measured wheel targets without Isaac Sim."""

    def __init__(self, node):
        self.node = node
        self.spec = json.loads((Path(get_package_share_directory('amr_description')) /
                                'config/compact_amr.json').read_text())
        self.wheels = self.spec['wheels']
        package = Path(get_package_share_directory('amr_mapping'))
        metadata = yaml.safe_load((package / 'map/map.yaml').read_text())
        self.pixels = np.array(Image.open(package / 'map' / metadata['image']))
        self.occupied = self.pixels < (1 - metadata['occupied_thresh']) * 255
        self.resolution = metadata['resolution']
        self.origin = metadata['origin'][:2]
        self.pose = np.zeros(3)
        self.angles = np.zeros(4)
        self.targets = np.zeros(4)
        self.last_target = time.monotonic()
        self.sim_ns = 10000000000
        self.commands = []
        self.maps = []
        self.paths = []
        self.poses = []
        self.monitor = []
        self.close_obstacle = False
        self.raw_command = None
        self.clock = node.create_publisher(Clock, '/clock', 10)
        self.joints = node.create_publisher(JointState, '/joint_states', 10)
        self.initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.raw = node.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        node.create_subscription(JointState, '/wheel_commands', self.receive_wheels, 10)
        node.create_subscription(Twist, '/cmd_vel', self.commands.append, 10)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.poses.append, 10)
        node.create_subscription(NavPath, '/plan', self.paths.append, 10)
        node.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.monitor.append, 10)
        node.create_subscription(OccupancyGrid, '/map', self.maps.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.static = StaticTransformBroadcaster(node)
        self.scanners = []
        transforms = []
        for name in ('lidar_front_left', 'lidar_rear_right'):
            sensor = self.spec['sensors'][name]
            self.scanners.append((name, sensor, node.create_publisher(LaserScan, sensor['topic'], 10)))
            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.child_frame_id = name + '_scan'
            transform.transform.translation.x, transform.transform.translation.y = sensor['xyz_m'][:2]
            transform.transform.translation.z = sensor['xyz_m'][2] + 0.025
            transform.transform.rotation.z = math.sin(sensor['rpy_rad'][2] / 2)
            transform.transform.rotation.w = math.cos(sensor['rpy_rad'][2] / 2)
            transforms.append(transform)
        self.static.sendTransform(transforms)
        xs, ys = (1, 1, -1, -1), (1, -1, 1, -1)
        hands = self.wheels['roller_handedness']
        matrix = np.array([[1, hand, -y * self.wheels['track_m'] / 2
                            + hand * x * self.wheels['wheelbase_m'] / 2]
                           for x, y, hand in zip(xs, ys, hands)]) / self.wheels['radius_m']
        self.forward = np.linalg.pinv(matrix)

    def receive_wheels(self, message):
        values = dict(zip(message.name, message.velocity))
        self.targets = np.array([values[n + '_wheel_joint'] for n in self.wheels['order']])
        self.last_target = time.monotonic()

    def stamp(self):
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = divmod(self.sim_ns, 1000000000)
        return stamp

    def scan(self, name, sensor):
        scan = LaserScan()
        scan.header.stamp = self.stamp()
        scan.header.frame_id = name + '_scan'
        scan.range_min, scan.range_max = sensor['range_m']
        scan.angle_min = math.radians(-sensor['horizontal_fov_deg'] / 2)
        scan.angle_increment = math.radians(sensor['horizontal_resolution_deg'])
        count = round(sensor['horizontal_fov_deg'] / sensor['horizontal_resolution_deg'])
        scan.angle_max = scan.angle_min + (count - 1) * scan.angle_increment
        scan.scan_time = 0.05
        angles = self.pose[2] + sensor['rpy_rad'][2] + scan.angle_min + np.arange(count) * scan.angle_increment
        sx, sy = sensor['xyz_m'][:2]
        ox = self.pose[0] + math.cos(self.pose[2]) * sx - math.sin(self.pose[2]) * sy
        oy = self.pose[1] + math.sin(self.pose[2]) * sx + math.cos(self.pose[2]) * sy
        distances = np.arange(scan.range_min, scan.range_max, self.resolution / 2)
        cols = np.floor((ox + np.cos(angles)[:, None] * distances - self.origin[0]) /
                        self.resolution).astype(int)
        rows = self.pixels.shape[0] - 1 - np.floor(
            (oy + np.sin(angles)[:, None] * distances - self.origin[1]) / self.resolution).astype(int)
        inside = ((cols >= 0) & (rows >= 0) & (cols < self.pixels.shape[1]) &
                  (rows < self.pixels.shape[0]))
        hits = inside & self.occupied[np.clip(rows, 0, self.pixels.shape[0] - 1),
                                      np.clip(cols, 0, self.pixels.shape[1] - 1)]
        ranges = np.where(hits.any(axis=1), distances[hits.argmax(axis=1)], scan.range_max)
        if self.close_obstacle and name == 'lidar_front_left':
            ranges[:] = 0.06
        scan.ranges = ranges.tolist()
        return scan

    def step(self):
        start = time.monotonic()
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.001)
        wheels = self.targets if start - self.last_target < 0.3 else np.zeros(4)
        velocity = self.forward @ wheels
        dt = 0.05
        mid_yaw = self.pose[2] + velocity[2] * dt / 2
        self.pose[0] += (math.cos(mid_yaw) * velocity[0] - math.sin(mid_yaw) * velocity[1]) * dt
        self.pose[1] += (math.sin(mid_yaw) * velocity[0] + math.cos(mid_yaw) * velocity[1]) * dt
        self.pose[2] += velocity[2] * dt
        self.angles += wheels * dt
        self.sim_ns += 50000000
        clock = Clock()
        clock.clock = self.stamp()
        self.clock.publish(clock)
        joint = JointState()
        joint.header.stamp = clock.clock
        joint.name = [n + '_wheel_joint' for n in self.wheels['order']]
        joint.position = ((self.angles + 2 * math.pi) % (4 * math.pi) - 2 * math.pi).tolist()
        joint.velocity = wheels.tolist()
        self.joints.publish(joint)
        for name, sensor, publisher in self.scanners:
            publisher.publish(self.scan(name, sensor))
        if self.raw_command is not None:
            self.raw.publish(self.raw_command)
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.001)
        time.sleep(max(0.001, dt - (time.monotonic() - start)))


def main():
    root = Path(tempfile.mkdtemp(prefix='amr-navigation-'))
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    print('Navigation test artifacts:', root, 'domain', os.environ['ROS_DOMAIN_ID'], flush=True)
    rclpy.init()
    node = rclpy.create_node('navigation_hardware_fixture')
    log_path = root / 'navigation.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'amr_navigation', 'navigation.launch.py', 'rviz:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            hardware = Hardware(node)

            def until(predicate, timeout=25):
                deadline = time.monotonic() + timeout
                while not predicate():
                    assert process.poll() is None, 'Navigation exited'
                    assert time.monotonic() < deadline, 'Timed out: ' + str(log_path)
                    hardware.step()

            def active(name):
                client = node.create_client(GetState, '/' + name + '/get_state')
                until(client.service_is_ready)
                deadline = time.monotonic() + 25
                while True:
                    future = client.call_async(GetState.Request())
                    until(future.done)
                    if future.result().current_state.id == 3:
                        break
                    assert time.monotonic() < deadline, name + ' did not activate'
                node.destroy_client(client)

            active('amcl')
            until(lambda: bool(hardware.maps))
            for _ in range(10):
                hardware.step()
            assert not hardware.buffer.can_transform('map', 'odom', Time()), 'AMCL skipped initial pose'
            assert np.max(np.abs(hardware.targets)) < 1e-6, 'Motion before operator initialization'
            until(lambda: node.count_subscribers('/initialpose') >= 1)
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.pose.pose.orientation.w = 1.0
            initial.pose.covariance[0] = initial.pose.covariance[7] = 0.04
            initial.pose.covariance[35] = 0.02
            for _ in range(3):
                initial.header.stamp = hardware.stamp()
                hardware.initial.publish(initial)
                hardware.step()
            until(lambda: bool(hardware.poses) and
                  hardware.buffer.can_transform('map', 'base_footprint', Time()))
            for name in ('controller_server', 'planner_server', 'bt_navigator', 'collision_monitor'):
                active(name)
            assert node.count_publishers('/cmd_vel') == 1
            params = node.create_client(GetParameters, '/amcl/get_parameters')
            until(params.service_is_ready)
            req = GetParameters.Request()
            req.names = ['robot_model_type', 'scan_topic']
            future = params.call_async(req)
            until(future.done)
            assert future.result().values[0].string_value == 'nav2_amcl::OmniMotionModel'
            assert future.result().values[1].string_value == '/scan/front'
            node.destroy_client(params)
            # Nav2's params_file must not replace the wheel odometry configuration.
            params = node.create_client(GetParameters, '/wheel_odometry/get_parameters')
            until(params.service_is_ready)
            req.names = ['position_wrap_period_rad']
            future = params.call_async(req)
            until(future.done)
            assert math.isclose(future.result().values[0].double_value, 4 * math.pi)
            node.destroy_client(params)

            navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            until(navigator.server_is_ready)

            def send_goal(x, y):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = hardware.stamp()
                goal.pose.pose.position.x = x
                goal.pose.pose.position.y = y
                goal.pose.pose.orientation.w = 1.0
                future = navigator.send_goal_async(goal)
                until(future.done)
                handle = future.result()
                assert handle.accepted, 'Goal was rejected'
                return handle

            goal = send_goal(0.0, 0.7)
            result = goal.get_result_async()
            until(result.done, timeout=45)
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED, result.result()
            assert hardware.paths and hardware.paths[-1].poses
            assert np.linalg.norm(hardware.pose[:2] - [0, 0.7]) < 0.22, hardware.pose
            assert any(abs(cmd.linear.y) > 0.02 for cmd in hardware.commands), 'No lateral command'
            goal_pose = hardware.pose.tolist()
            print('PASS native navigation: map load, AMCL /initialpose, Omni model, '
                  'path, lateral goal and C++ wheel/odom pipeline', hardware.pose.tolist(), flush=True)

            goal = send_goal(0.9, 0.7)
            for _ in range(8):
                hardware.step()
            cancel = goal.cancel_goal_async()
            until(cancel.done)
            assert cancel.result().goals_canceling, 'Goal cancellation rejected'
            result = goal.get_result_async()
            until(result.done)
            assert result.result().status == GoalStatus.STATUS_CANCELED
            for _ in range(20):
                hardware.step()
            assert np.max(np.abs(hardware.targets)) < 0.02, hardware.targets
            print('PASS cancellation: action canceled and wheel targets stopped', flush=True)

            # Exercise native Collision Monitor separately after the goal is canceled.
            command = Twist()
            command.linear.x = 0.1
            hardware.raw_command = command
            until(lambda: hardware.commands and hardware.commands[-1].linear.x > 0.05)
            hardware.close_obstacle = True
            for _ in range(15):
                hardware.step()
            assert hardware.monitor and hardware.monitor[-1].action_type != CollisionMonitorState.DO_NOTHING
            cmd = hardware.commands[-1]
            assert max(abs(cmd.linear.x), abs(cmd.linear.y), abs(cmd.angular.z)) < 1e-6
            assert np.max(np.abs(hardware.targets)) < 0.02
            hardware.raw_command = Twist()
            for _ in range(3):
                hardware.step()
            print('PASS close obstacle: Collision Monitor stops command and measured wheel targets', flush=True)
            (root / 'result.json').write_text(json.dumps({
                'goal_pose': goal_pose, 'final_pose': hardware.pose.tolist(), 'lateral_commands': sum(
                    abs(c.linear.y) > 0.02 for c in hardware.commands),
                'goal_succeeded': True, 'cancel_stopped': True, 'collision_stopped': True}, indent=2))
        except Exception:
            print(log_path.read_text()[-14000:], flush=True)
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
            rclpy.shutdown()


if __name__ == '__main__':
    main()
