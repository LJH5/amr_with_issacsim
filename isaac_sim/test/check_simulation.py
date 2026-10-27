#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check playback, acceleration, units and cleanup without starting Isaac Sim."""

import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
from drive import ramp_command
from simulation import run_loop


SPEC = json.loads((SIM_DIR.parent / 'ros2_ws/src/amr_description/config/compact_amr.json').read_text())


class FakeDrive:
    def __init__(self):
        self.value = None
        self.valid = True

    def __bool__(self):
        return self.valid

    def GetTargetVelocityAttr(self):
        return self

    def Set(self, value):
        if not self.valid:
            raise RuntimeError('expired USD drive')
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
    values = dict(vx=0.3, vy=0.0, wz=0.0, drive_seconds=2.0, steps=0,
                  headless=False, scene=None, ros2=True)
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

    def test_stage_replacement_keeps_application_running(self):
        playback = Playback([0.0, 0.1, 0.2])
        context = SimpleNamespace(stage=object())
        original_update = playback.update

        def open_another_stage():
            original_update()
            context.stage = object()
            for drive in playback.drives:
                drive.valid = False

        playback.update = open_another_stage
        run_loop(playback, playback, options(), SPEC, playback.drives,
                 get_stage=lambda: context.stage)
        self.assertEqual(len(playback.samples), 3)
        self.assertTrue(playback.stopped)

    def test_closing_stage_on_last_update_skips_stale_cleanup(self):
        playback = Playback([0.0])
        context = SimpleNamespace(stage=object())
        original_update = playback.update

        def close_stage():
            original_update()
            context.stage = None
            for drive in playback.drives:
                drive.valid = False

        playback.update = close_stage
        run_loop(playback, playback, options(steps=1), SPEC, playback.drives,
                 get_stage=lambda: context.stage)
        self.assertTrue(playback.stopped)

    def test_deleted_drive_does_not_close_playback(self):
        playback = Playback([0.0, 0.1])
        playback.drives[0].valid = False
        run_loop(playback, playback, options(), SPEC, playback.drives)
        self.assertEqual(len(playback.samples), 2)

    def test_default_scene_and_new_scene_options(self):
        from cli import arguments
        with patch.object(sys, 'argv', ['run_amr.py']):
            args = arguments()
        self.assertEqual(args.scene, SIM_DIR / 'omni_amr.usd')
        self.assertTrue(args.ros2)
        with patch.object(sys, 'argv', ['run_amr.py', '--new-scene', '--no-ros2']):
            args = arguments()
        self.assertIsNone(args.scene)
        self.assertFalse(args.ros2)

    def test_saved_scene_is_not_regenerated_or_exported(self):
        from simulation import run
        stage = Mock()
        omni = ModuleType('omni')
        omni.timeline = ModuleType('omni.timeline')
        omni.timeline.get_timeline_interface = Mock()
        omni.usd = ModuleType('omni.usd')
        omni.usd.get_context = Mock()
        modules = {'omni': omni, 'omni.timeline': omni.timeline, 'omni.usd': omni.usd}
        hooks = {name: Mock() for name in (
            'create_stage', 'import_robot', 'add_environment', 'configure_drives', 'run_loop')}
        hooks['create_stage'].return_value = stage
        args = options(scene=SIM_DIR / 'omni_amr.usd')
        with patch.dict(sys.modules, modules), patch.multiple('simulation', **hooks):
            run(Mock(), args, SPEC, SIM_DIR, SIM_DIR.parent / 'ros2_ws/src/amr_description')
        hooks['create_stage'].assert_called_once_with(False, args.scene, True)
        hooks['import_robot'].assert_not_called()
        hooks['add_environment'].assert_not_called()
        stage.GetRootLayer.assert_not_called()
        hooks['run_loop'].assert_called_once()


if __name__ == '__main__':
    unittest.main()
