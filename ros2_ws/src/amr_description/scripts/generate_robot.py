#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Generate the prototype URDF using only Python's standard library."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def numbers(values):
    return ' '.join(f'{value:.10g}' for value in values)


def origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    ET.SubElement(parent, 'origin', xyz=numbers(xyz), rpy=numbers(rpy))


def body(robot, name, mass, shape, dimensions, color):
    link = ET.SubElement(robot, 'link', name=name)
    if shape == 'box':
        x, y, z = dimensions
        inertia = [mass * (y*y + z*z) / 12,
                   mass * (x*x + z*z) / 12,
                   mass * (x*x + y*y) / 12]
        attrs = {'size': numbers(dimensions)}
    else:
        radius, length = dimensions
        transverse = mass * (3*radius*radius + length*length) / 12
        inertia = [transverse, transverse, mass * radius*radius / 2]
        attrs = {'radius': str(radius), 'length': str(length)}
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'inertia', ixx=str(inertia[0]),
                  iyy=str(inertia[1]), izz=str(inertia[2]),
                  ixy='0', ixz='0', iyz='0')
    for kind in ('visual', 'collision'):
        element = ET.SubElement(link, kind)
        geometry = ET.SubElement(element, 'geometry')
        ET.SubElement(geometry, shape, **attrs)
        if kind == 'visual':
            material = ET.SubElement(element, 'material', name=name + '_material')
            ET.SubElement(material, 'color', rgba=color)
    return link


def joint(robot, name, parent, child, xyz, rpy=(0, 0, 0), limits=None):
    element = ET.SubElement(robot, 'joint', name=name,
                            type='continuous' if limits else 'fixed')
    ET.SubElement(element, 'parent', link=parent)
    ET.SubElement(element, 'child', link=child)
    origin(element, xyz, rpy)
    if limits:
        # All cylindrical bodies use their local Z as the rotation axis.
        ET.SubElement(element, 'axis', xyz='0 0 1')
        ET.SubElement(element, 'limit', effort=str(limits[0]), velocity=str(limits[1]))
        ET.SubElement(element, 'dynamics', damping='0', friction='0')
    return element


def build_robot(spec):
    robot = ET.Element('robot', name=spec['name'])
    chassis, wheels = spec['chassis'], spec['wheels']
    height = chassis['center_height_m']
    ET.SubElement(robot, 'link', name='base_footprint')
    body(robot, 'base_link', chassis['mass_kg'], 'box',
         chassis['size_m'], '0.12 0.28 0.42 1')
    joint(robot, 'base_joint', 'base_footprint', 'base_link', (0, 0, height))

    count = wheels['rollers_per_wheel']
    radius = wheels['radius_m']
    for name, xsign, ysign, hand in zip(
            wheels['order'], (1, 1, -1, -1), (1, -1, 1, -1),
            wheels['roller_handedness']):
        wheel = name + '_wheel'
        body(robot, wheel, wheels['hub_mass_kg'], 'cylinder',
             (wheels['hub_radius_m'], wheels['hub_width_m']), '0.25 0.25 0.25 1')
        # Rx(-pi/2) maps the hub's local +Z axis to the robot's +Y.
        joint(robot, name + '_wheel_joint', 'base_link', wheel,
              (xsign * wheels['wheelbase_m'] / 2,
               ysign * wheels['track_m'] / 2, radius - height),
              (-math.pi / 2, 0, 0),
              (wheels['joint_effort_limit_nm'], wheels['joint_velocity_limit_rad_s']))
        for index in range(count):
            theta = 2 * math.pi * index / count
            roller = f'{name}_roller_{index}'
            body(robot, roller, wheels['roller_mass_kg'], 'cylinder',
                 (wheels['roller_radius_m'], wheels['roller_length_m']),
                 '0.08 0.08 0.08 1')
            distance = radius - wheels['roller_radius_m']
            # Tangent/helical axis in hub coordinates, tilted toward local Z.
            angle = math.radians(wheels['roller_angle_deg'])
            axis = (-math.sin(theta) * math.cos(angle),
                    math.cos(theta) * math.cos(angle), hand * math.sin(angle))
            # Rz(yaw) Ry(pitch) maps the roller's +Z onto the helical axis.
            pitch = math.acos(axis[2])
            yaw = math.atan2(axis[1], axis[0])
            joint(robot, roller + '_joint', wheel, roller,
                  (distance * math.cos(theta), distance * math.sin(theta), 0),
                  (0, pitch, yaw), (0.0, 200.0))

    for name, sensor in spec['sensors'].items():
        if name.startswith('lidar'):
            shape, size, color = 'cylinder', (0.025, 0.04), '0.9 0.6 0.05 1'
        elif name.startswith('camera'):
            shape, size, color = 'box', (0.025, 0.09, 0.025), '0.1 0.1 0.1 1'
        else:
            shape, size, color = 'box', (0.025, 0.025, 0.012), '0.2 0.8 0.3 1'
        body(robot, name + '_link', sensor['mass_kg'], shape, size, color)
        x, y, z = sensor['xyz_m']
        joint(robot, name + '_joint', 'base_link', name + '_link',
              (x, y, z - height), sensor['rpy_rad'])
    ET.SubElement(robot, 'link', name='camera_front_optical_frame')
    joint(robot, 'camera_front_optical_joint', 'camera_front_link',
          'camera_front_optical_frame', (0, 0, 0), (-math.pi / 2, 0, -math.pi / 2))
    return robot


def main():
    spec = json.loads((PACKAGE / 'config/compact_amr.json').read_text())
    robot = build_robot(spec)
    ET.indent(robot)
    output = PACKAGE / 'urdf/compact_amr.urdf'
    output.parent.mkdir(exist_ok=True)
    output.write_text('<?xml version="1.0"?>\n'
                      '<!-- Copyright 2026 LJH5; SPDX-License-Identifier: Apache-2.0 -->\n'
                      '<!-- Generated by scripts/generate_robot.py; edit config/compact_amr.json. -->\n'
                      + ET.tostring(robot, encoding='unicode') + '\n')
    print(output)


if __name__ == '__main__':
    main()
