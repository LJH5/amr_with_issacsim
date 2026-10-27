# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Body command limits and inverse kinematics; no simulator imports."""

import math


def wheel_targets(spec, vx, vy, wz):
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError('Velocity commands must be finite')
    motion, wheels = spec['motion_targets'], spec['wheels']
    speed = math.hypot(vx, vy)
    if speed > motion['max_planar_speed_m_s']:
        scale = motion['max_planar_speed_m_s'] / speed
        vx, vy = vx * scale, vy * scale
    maximum = motion['max_yaw_rate_rad_s']
    wz = max(-maximum, min(maximum, wz))
    targets = []
    for xsign, ysign, hand in zip((1, 1, -1, -1), (1, -1, 1, -1),
                                 wheels['roller_handedness']):
        x = xsign * wheels['wheelbase_m'] / 2
        y = ysign * wheels['track_m'] / 2
        targets.append((vx - wz*y + hand*(vy + wz*x)) / wheels['radius_m'])
    peak = max(abs(value) for value in targets)
    if peak > wheels['joint_velocity_limit_rad_s']:
        scale = wheels['joint_velocity_limit_rad_s'] / peak
        targets = [value * scale for value in targets]
    return targets
