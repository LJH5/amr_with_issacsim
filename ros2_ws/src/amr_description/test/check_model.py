#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check the generated tree, mass, roller geometry, envelope and speed budget."""

import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
module_spec = importlib.util.spec_from_file_location(
    'generator', PACKAGE / 'scripts/generate_robot.py')
generator = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(generator)
spec = json.loads((PACKAGE / 'config/compact_amr.json').read_text())
robot = ET.parse(PACKAGE / 'urdf/compact_amr.urdf').getroot()
generated = generator.build_robot(spec)
for element in robot.iter():
    element.text = element.tail = None
assert ET.tostring(robot) == ET.tostring(generated), 'URDF is stale; regenerate it'

links = {link.attrib['name']: link for link in robot.findall('link')}
joints = robot.findall('joint')
children = [joint.find('child').attrib['link'] for joint in joints]
assert len(children) == len(set(children))
assert set(links) - set(children) == {'base_footprint'}
assert len(joints) == len(links) - 1
reachable = {'base_footprint'}
while True:
    expanded = reachable | {
        joint.find('child').attrib['link'] for joint in joints
        if joint.find('parent').attrib['link'] in reachable}
    if expanded == reachable:
        break
    reachable = expanded
assert reachable == set(links), 'Disconnected or cyclic link tree'
mass = sum(float(item.attrib['value']) for item in robot.findall('link/inertial/mass'))
assert math.isclose(mass, 20.0), mass
for inertia in robot.findall('link/inertial/inertia'):
    diagonal = [float(inertia.attrib[key]) for key in ('ixx', 'iyy', 'izz')]
    assert all(value > 0 for value in diagonal)
    assert all(value <= sum(diagonal) - value + 1e-10 for value in diagonal)

wheels = spec['wheels']
r = wheels['radius_m']
offset = (wheels['wheelbase_m'] + wheels['track_m']) / 2
motion = spec['motion_targets']
# Simultaneous worst-case diagonal translation and yaw must fit each motor.
required = (math.sqrt(2) * motion['max_planar_speed_m_s']
            + offset * motion['max_yaw_rate_rad_s']) / r
assert required <= wheels['joint_velocity_limit_rad_s']
assert motion['max_planar_speed_m_s'] == 1.5

for name, hand, ysign in zip(wheels['order'], wheels['roller_handedness'], (1, -1, 1, -1)):
    roller_joints = [joint for joint in joints
                     if joint.find('parent').attrib['link'] == name + '_wheel']
    assert len(roller_joints) == wheels['rollers_per_wheel']
    # At the bottom contact, hub-local +Y points downward after Rx(-pi/2).
    contact = roller_joints[wheels['rollers_per_wheel'] // 4]
    _, pitch, yaw = map(float, contact.find('origin').attrib['rpy'].split())
    axis_hub = (math.cos(yaw) * math.sin(pitch),
                math.sin(yaw) * math.sin(pitch), math.cos(pitch))
    axis_robot = (axis_hub[0], axis_hub[2], -axis_hub[1])
    for vx, vy, wz in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
        x = wheels['wheelbase_m'] / 2 * (1 if name.startswith('front') else -1)
        y = wheels['track_m'] / 2 * ysign
        omega = (vx - wz*y + hand*(vy + wz*x)) / r
        # Contact velocity is parallel to the passive roller axis.
        contact_velocity = (vx - wz*y - r*omega, vy + wz*x)
        cross = contact_velocity[0]*axis_robot[1] - contact_velocity[1]*axis_robot[0]
        assert abs(cross) < 1e-8, (name, vx, vy, wz)

width = wheels['track_m'] + 2 * (
    wheels['roller_length_m']/2 + wheels['roller_radius_m'])
assert width <= spec['design']['footprint_size_m'][1]
for name, sensor in spec['sensors'].items():
    assert name + '_link' in links
    assert sensor['xyz_m'][2] <= spec['design']['height_envelope_m']
assert 'camera_front_optical_frame' in links
print(f'PASS: {len(links)} links, {len(joints)} joints, {mass:.1f} kg; '
      f'wheel demand {required:.2f} < {wheels["joint_velocity_limit_rad_s"]:.1f} rad/s')
