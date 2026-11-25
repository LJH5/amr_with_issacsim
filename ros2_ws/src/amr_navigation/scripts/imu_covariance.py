#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Supply variances for IMU drivers that leave orientation and gyro covariance unknown."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuCovariance(Node):
    """Forward IMU measurements with fallback covariances."""

    def __init__(self):
        super().__init__('imu_covariance')
        self.variances = {}
        for field, parameter in (
            ('angular_velocity_covariance', 'fallback_angular_variance'),
            ('orientation_covariance', 'fallback_orientation_variance'),
        ):
            variance = self.declare_parameter(parameter, 0.0025).value
            if not math.isfinite(variance) or variance <= 0:
                raise ValueError(f'{parameter} must be finite and positive')
            self.variances[field] = variance
        self.publisher = self.create_publisher(
            Imu, '/imu/with_covariance', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.forward, qos_profile_sensor_data)

    def forward(self, message):
        """Fill the all-zero covariance while preserving known and unavailable values."""
        for field, variance in self.variances.items():
            covariance = getattr(message, field)
            if not any(covariance):
                for index in (0, 4, 8):
                    covariance[index] = variance
        self.publisher.publish(message)


def main():
    """Run the sensor adapter until launch shuts down."""
    rclpy.init()
    node = ImuCovariance()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
