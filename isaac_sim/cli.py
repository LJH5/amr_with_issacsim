# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Import the shared URDF and run a small, standalone Isaac Sim test scene."""

import argparse
import math


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--vx', type=float, default=0.0, help='Forward velocity, m/s')
    parser.add_argument('--vy', type=float, default=0.0, help='Leftward velocity, m/s')
    parser.add_argument('--wz', type=float, default=0.0, help='CCW yaw rate, rad/s')
    parser.add_argument('--drive-seconds', type=float, default=2.0,
                        help='Stop driving after this many simulated seconds')
    parser.add_argument('--steps', type=int, default=0,
                        help='Exit after N application updates; 0 runs until closed')
    args = parser.parse_args()
    if not math.isfinite(args.drive_seconds) or args.drive_seconds < 0 or args.steps < 0:
        parser.error('drive-seconds and steps must be nonnegative and finite')
    return args
