#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Camera projection, message layout/units and invalid-input checks without GPU."""

import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from camera import camera_parameters, OPTICAL_FRAME
from check_topics import image_values, camera_info_values

CONFIG = json.loads((SIM_DIR.parent /
                     'ros2_ws/src/amr_description/config/compact_amr.json').read_text())['sensors']['camera_front']


class CameraChecks(unittest.TestCase):
    def test_projection_and_rate(self):
        values = camera_parameters(CONFIG, 60)
        self.assertEqual(values['frame_skip'], 1)
        self.assertAlmostEqual(values['horizontal_aperture'] / values['focal_length'],
                               2 * math.tan(math.radians(87) / 2))
        for overrides in ({'rate_hz': 0}, {'rate_hz': 29}, {'horizontal_fov_deg': 180},
                          {'resolution_px': [0, 480]}, {'depth_range_m': [10, 0.2]}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                camera_parameters(dict(CONFIG, **overrides), 60)

    def test_depth_endianness_and_padding(self):
        config = dict(CONFIG, resolution_px=[2, 2])
        header = SimpleNamespace(frame_id=OPTICAL_FRAME, stamp=SimpleNamespace(sec=1, nanosec=0))
        for endian, prefix in ((False, '<'), (True, '>')):
            buffer = bytearray(24)  # two rows padded to 12 bytes each
            struct.pack_into(prefix + 'f', buffer, 16, 1.4)
            image = SimpleNamespace(header=header, width=2, height=2, encoding='32FC1',
                                    step=12, is_bigendian=endian, data=buffer)
            self.assertAlmostEqual(image_values(image, 'depth', config)['center_depth_m'], 1.4)
            image.data = buffer[:8]
            with self.assertRaises(ValueError):
                image_values(image, 'depth', config)

    def test_camera_calibration_and_color(self):
        header = SimpleNamespace(frame_id=OPTICAL_FRAME, stamp=SimpleNamespace(sec=1, nanosec=0))
        focal = 640 / (2 * math.tan(math.radians(87) / 2))
        info = SimpleNamespace(header=header, width=640, height=480,
                               k=[focal, 0, 320, 0, focal, 240, 0, 0, 1], p=[0.0] * 12)
        self.assertAlmostEqual(camera_info_values(info, CONFIG)['fx'], focal)
        image = SimpleNamespace(header=header, width=2, height=2, encoding='rgb8', step=6,
                                data=bytes([255, 0, 0] * 4))
        self.assertEqual(image_values(image, 'color', dict(CONFIG, resolution_px=[2, 2]))['center_rgb'], [255, 0, 0])
        info.k[0] = 0
        with self.assertRaises(ValueError):
            camera_info_values(info, CONFIG)


if __name__ == '__main__':
    unittest.main()
