#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Offline receipt-counter/watchdog checks, independent of DDS and Isaac Sim."""

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_commands import TimedTwist


class CommandChecks(unittest.TestCase):
    def test_no_new_message_means_stop(self):
        buffer = TimedTwist(0.25, initial_count=7)
        self.assertEqual(buffer.read(7, [1, 0, 0], [0, 0, 1], 0), [0]*3)

    def test_twist_axes_and_timeout_boundary(self):
        buffer = TimedTwist(0.25)
        self.assertEqual(buffer.read(1, [0.1, 0.2, 99], [99, 99, 0.3], 0), [0.1, 0.2, 0.3])
        self.assertEqual(buffer.read(1, [0, 0, 0], [0, 0, 0], 0.24), [0.1, 0.2, 0.3])
        self.assertEqual(buffer.read(1, [1, 1, 1], [1, 1, 1], 0.25), [0]*3)

    def test_identical_messages_refresh_receipt(self):
        buffer = TimedTwist(0.25)
        buffer.read(1, [0.2, 0, 0], [0, 0, 0], 0)
        buffer.read(2, [0.2, 0, 0], [0, 0, 0], 0.2)
        self.assertEqual(buffer.read(2, None, None, 0.3), [0.2, 0, 0])

    def test_invalid_new_message_stops_immediately(self):
        for linear, angular in (([float('nan'), 0, 0], [0, 0, 0]),
                                ([0, 0, 0], [0, 0, float('inf')]),
                                (None, None), ([], [])):
            buffer = TimedTwist(0.25)
            buffer.read(1, [0.2, 0, 0], [0, 0, 0], 0)
            self.assertEqual(buffer.read(2, linear, angular, 0.1), [0]*3)

    def test_reset_and_counter_rewind_require_fresh_message(self):
        buffer = TimedTwist(0.25)
        buffer.read(5, [0.2, 0, 0], [0, 0, 0], 0)
        self.assertEqual(buffer.read(0, [0.2, 0, 0], [0, 0, 0], 0.1), [0]*3)
        self.assertEqual(buffer.read(1, [0.2, 0, 0], [0, 0, 0], 0.2), [0.2, 0, 0])
        buffer.reset(1)
        self.assertEqual(buffer.read(1, [0.2, 0, 0], [0, 0, 0], 0.21), [0]*3)


if __name__ == '__main__':
    unittest.main()
