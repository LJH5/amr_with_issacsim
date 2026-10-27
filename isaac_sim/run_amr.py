#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Prepare the model and launch the standalone AMR simulation."""

import json
from pathlib import Path
import runpy

from cli import arguments
from mecanum import wheel_targets
from simulation import run


SIM_DIR = Path(__file__).resolve().parent
DESCRIPTION = SIM_DIR.parent / "ros2_ws/src/amr_description"


def main():
    args = arguments()
    spec = json.loads((DESCRIPTION / 'config/compact_amr.json').read_text())
    wheel_targets(spec, args.vx, args.vy, args.wz)  # Validate before starting Kit.
    runpy.run_path(str(DESCRIPTION / 'scripts/generate_robot.py'), run_name='__main__')
    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        raise SystemExit('Isaac Sim 전용 Python으로 실행하세요. README.md 참고.') from error
    app = SimulationApp({'headless': args.headless})
    try:
        run(app, args, spec, SIM_DIR, DESCRIPTION)
    finally:
        app.close()


if __name__ == '__main__':
    main()
