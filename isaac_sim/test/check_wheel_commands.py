#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Receipt watchdog, joint-name mapping and rejected command regression checks."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_commands import WheelCommandBuffer


class WheelCommandsChecks(unittest.TestCase):
    def test_receipt_and_name_mapping(self):
        buffer = WheelCommandBuffer(['fl', 'fr', 'rl', 'rr'], 0.25, 35)
        names, velocities = ['rr', 'rl', 'fr', 'fl'], [4, 3, 2, 1]
        self.assertEqual(buffer.read(0, names, velocities, 0), [0] * 4)
        self.assertEqual(buffer.read(1, names, velocities, 1), [1, 2, 3, 4])
        self.assertEqual(buffer.read(1, names, velocities, 1.26), [0] * 4)
        self.assertEqual(buffer.read(2, names, velocities, 1.27), [1, 2, 3, 4])
        self.assertEqual(buffer.read(0, names, velocities, 1.28), [0] * 4)

    def test_invalid_commands_stop(self):
        for names, values in ((['fl', 'fl', 'rl', 'rr'], [1] * 4),
                              (['fl', 'fr', 'rl', 'rr'], [1, 2]),
                              (['fl', 'fr', 'rl', 'rr'], [float('nan')] * 4),
                              (['fl', 'fr', 'rl', 'rr'], [36] * 4)):
            buffer = WheelCommandBuffer(['fl', 'fr', 'rl', 'rr'], 0.25, 35)
            self.assertEqual(buffer.read(1, names, values, 0), [0] * 4)


if __name__ == '__main__':
    unittest.main()
