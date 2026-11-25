#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Wait for costmap transforms before starting the navigation lifecycle manager."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    """Exit successfully as soon as the local and global costmap TFs exist."""
    rclpy.init()
    node = rclpy.create_node('navigation_tf_ready')
    frames = node.declare_parameter(
        'costmap_frames', ['odom', 'base_footprint', 'map', 'base_footprint']).value
    pairs = list(dict.fromkeys(zip(frames[::2], frames[1::2], strict=True)))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        while rclpy.ok():
            missing = [f'{target} → {source}' for target, source in pairs
                       if not buffer.can_transform(target, source, Time())]
            if not missing:
                node.get_logger().info('Costmap TFs ready. Starting Nav2 lifecycle.')
                return 0
            node.get_logger().info(
                'Waiting for ' + ', '.join(missing) +
                '. Play the simulator and set 2D Pose Estimate in RViz.',
                throttle_duration_sec=5.0)
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
