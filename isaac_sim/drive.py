# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Wheel setup, acceleration limiting and USD velocity commands."""

import math

from mecanum import wheel_targets


def configure_drives(stage, spec):
    from pxr import PhysxSchema, UsdPhysics

    joints = {prim.GetName(): prim for prim in stage.Traverse()
              if prim.IsA(UsdPhysics.RevoluteJoint)}
    drives = []
    for name in spec['wheels']['order']:
        prim = joints[name + '_wheel_joint']
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(200.0)
        drive.CreateMaxForceAttr(spec['wheels']['joint_effort_limit_nm'])
        drive.CreateTargetVelocityAttr(0.0)
        drives.append(drive)
    for name, prim in joints.items():
        if '_roller_' in name:
            # Rollers must remain passive: remove importer-created servos.
            prim.RemoveAPI(UsdPhysics.DriveAPI, 'angular')
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
    return drives


def ramp_command(previous, target, dt, motion):
    if target == [0.0]*3:
        previous = target  # Stop immediately when the command window expires.
    else:
        # Limit the planar acceleration as a vector, including diagonals.
        delta = [target[i] - previous[i] for i in (0, 1)]
        distance = math.hypot(*delta)
        limit = motion['max_planar_acceleration_m_s2'] * dt
        scale = min(1.0, limit / distance) if distance else 1.0
        previous[:2] = [previous[i] + delta[i]*scale for i in (0, 1)]
        limit = motion['max_yaw_acceleration_rad_s2'] * dt
        previous[2] += max(-limit, min(limit, target[2] - previous[2]))
    return previous


def apply_command(drives, spec, command):
    velocities = wheel_targets(spec, *command)
    for drive, velocity in zip(drives, velocities):
        # USD angular drives use degrees/s; inverse kinematics uses rad/s.
        drive.GetTargetVelocityAttr().Set(math.degrees(velocity))
