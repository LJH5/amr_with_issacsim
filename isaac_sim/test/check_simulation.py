#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check playback, acceleration, units and cleanup without starting Isaac Sim."""

import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from drive import ramp_command
from simulation import run_loop


SPEC = json.loads((SIM_DIR.parent / 'ros2_ws/src/amr_description/config/compact_amr.json').read_text())


class FakeDrive:
    def __init__(self):
        self.value = None

    def GetTargetVelocityAttr(self):
        return self

    def Set(self, value):
        self.value = value


class Playback:
    """Supply application/timeline methods and record commanded wheel speeds."""

    def __init__(self, times, fail_at=None):
        self.times = times
        self.fail_at = fail_at
        self.drives = [FakeDrive() for _ in range(4)]
        self.samples = []
        self.stopped = False
        self.playing = False

    def play(self):
        self.playing = True

    def stop(self):
        self.stopped = True

    def is_running(self):
        return len(self.samples) < len(self.times)

    def get_current_time(self):
        return self.times[len(self.samples)]

    def update(self):
        self.samples.append(tuple(drive.value for drive in self.drives))
        if len(self.samples) == self.fail_at:
            raise RuntimeError('simulated application failure')


def options(**overrides):
    values = dict(vx=0.3, vy=0.0, wz=0.0, drive_seconds=2.0, steps=0)
    values.update(overrides)
    return SimpleNamespace(**values)


class SimulationChecks(unittest.TestCase):
    def test_diagonal_acceleration_and_yaw(self):
        command = ramp_command([0.0]*3, [1.0, 1.0, 1.0], 0.1, SPEC['motion_targets'])
        self.assertAlmostEqual(math.hypot(*command[:2]), 0.05)
        self.assertAlmostEqual(command[2], 0.1)

    def test_deadline_and_degree_conversion(self):
        playback = Playback([0.0, 0.1, 0.2, 0.3])
        run_loop(playback, playback, options(drive_seconds=0.2), SPEC, playback.drives)
        expected = math.degrees(0.05 / SPEC['wheels']['radius_m'])
        for value in playback.samples[1]:
            self.assertAlmostEqual(value, expected)
        self.assertEqual(playback.samples[0], (0.0,)*4)
        self.assertEqual(playback.samples[2:], [(0.0,)*4]*2)
        self.assertTrue(playback.playing and playback.stopped)

    def test_step_limit_stops_drives(self):
        playback = Playback([0.0, 0.1, 0.2])
        run_loop(playback, playback, options(steps=2), SPEC, playback.drives)
        self.assertEqual(len(playback.samples), 2)
        self.assertEqual([drive.value for drive in playback.drives], [0.0]*4)
        self.assertTrue(playback.stopped)

    def test_failure_stops_drives_and_propagates(self):
        playback = Playback([0.0, 0.1, 0.2], fail_at=2)
        with self.assertRaisesRegex(RuntimeError, 'simulated application failure'):
            run_loop(playback, playback, options(), SPEC, playback.drives)
        self.assertEqual([drive.value for drive in playback.drives], [0.0]*4)
        self.assertTrue(playback.stopped)

    def test_clock_jump_clamps_acceleration_step(self):
        playback = Playback([0.0, 10.0, 0.0])
        run_loop(playback, playback, options(drive_seconds=20), SPEC, playback.drives)
        self.assertEqual(playback.samples[1], playback.samples[2])
        self.assertAlmostEqual(playback.samples[1][0], math.degrees(0.05 / 0.075))

    def test_entry_import_does_not_initialize_isaac(self):
        import run_amr
        self.assertTrue(callable(run_amr.main))
        self.assertNotIn('isaacsim', sys.modules)
        self.assertNotIn('omni.kit.app', sys.modules)


if __name__ == '__main__':
    unittest.main()
