#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Offline wheel-command checks; does not import Isaac Sim."""

import importlib.util
import json
import math
from pathlib import Path


SIM_DIR = Path(__file__).resolve().parents[1]
module_spec = importlib.util.spec_from_file_location('mecanum', SIM_DIR / 'mecanum.py')
module = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(module)
spec = json.loads((SIM_DIR.parent / 'ros2_ws/src/amr_description/config/compact_amr.json').read_text())

assert module.wheel_targets(spec, 0, 0, 0) == [0]*4
assert all(math.isclose(value, 4.0) for value in module.wheel_targets(spec, 0.3, 0, 0))
assert module.wheel_targets(spec, 0, 0.3, 0) == [-4.0, 4.0, 4.0, -4.0]
rotation = module.wheel_targets(spec, 0, 0, 1)
assert rotation[0] < 0 < rotation[1] and rotation[2] < 0 < rotation[3]
diagonal = module.wheel_targets(spec, 1.5, 1.5, 0)
assert math.isclose(diagonal[1], math.sqrt(2)*1.5/0.075)
assert diagonal[0] == diagonal[3] == 0
for vx, vy, wz in ((1.5, 1.5, 1.2), (100, -100, -100), (-1.5, 1.5, 1.2)):
    assert max(map(abs, module.wheel_targets(spec, vx, vy, wz))) <= 35.0
for value in (float('nan'), float('inf')):
    try:
        module.wheel_targets(spec, value, 0, 0)
    except ValueError:
        pass
    else:
        raise AssertionError('Non-finite velocity was accepted')
print('PASS: stop / forward / lateral / yaw / diagonal speed limit / invalid inputs')
