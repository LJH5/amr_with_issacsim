#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Offline checks for scan validation and invalid sensor configuration."""

import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from check_topics import scan_values
from lidar import scan_metadata, validate_lidar_spec

SPEC = json.loads((SIM_DIR.parent / 'ros2_ws/src/amr_description/config/compact_amr.json').read_text())


class LidarChecks(unittest.TestCase):
    def setUp(self):
        self.name = 'lidar_front_left'
        self.config = SPEC['sensors'][self.name]
        self.scan = SimpleNamespace(
            header=SimpleNamespace(frame_id=self.name + '_scan',
                                   stamp=SimpleNamespace(sec=1, nanosec=0)),
            angle_min=math.radians(-135), angle_max=math.radians(134.5),
            angle_increment=math.radians(0.5), range_min=0.05, range_max=20.0,
            scan_time=0.05, time_increment=0.05 / 540,
            ranges=[math.inf] * 540)

    def test_horizontal_scan_and_empty_space(self):
        validate_lidar_spec(self.config)
        self.scan.ranges[270] = 1.1
        result = scan_values(self.scan, self.name, self.config)
        self.assertEqual(result['hits'], 1)
        self.assertEqual(result['forward_m'], 1.1)
        self.assertEqual(result['beams'], 540)
        metadata = scan_metadata(self.config)
        self.assertAlmostEqual(metadata['time_increment'], 0.5 / (360 * 20))
        self.assertTrue(all(math.isfinite(value) for value in metadata.values()))

    def test_bad_scan_is_rejected(self):
        for attribute, value in (('ranges', [1.0]), ('angle_increment', 0),
                                 ('angle_min', 0), ('angle_max', math.pi),
                                 ('scan_time', 0), ('time_increment', -1),
                                 ('range_max', 30), ('ranges', [math.nan] * 540)):
            with self.subTest(attribute=attribute, value=str(value)[:30]):
                scan = copy.deepcopy(self.scan)
                setattr(scan, attribute, value)
                with self.assertRaises(ValueError):
                    scan_values(scan, self.name, self.config)
        self.scan.header.frame_id = 'base_link'
        with self.assertRaises(ValueError):
            scan_values(self.scan, self.name, self.config)

    def test_invalid_parameters(self):
        for key, value in (('rate_hz', 0), ('rate_hz', math.nan),
                           ('range_m', [20, 0.05]), ('horizontal_resolution_deg', 0),
                           ('horizontal_resolution_deg', 270),
                           ('horizontal_fov_deg', 361), ('horizontal_resolution_deg', 7)):
            with self.subTest(key=key, value=value):
                config = dict(self.config, **{key: value})
                with self.assertRaises(ValueError):
                    validate_lidar_spec(config)


if __name__ == '__main__':
    unittest.main()
