#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Compare graph world pose with PhysX at an offset and rotated spawn."""
import json
import math
from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        import omni.graph.core as og
        import omni.physx
        import omni.timeline
        from pxr import Gf, UsdGeom
        from drive import apply_command, configure_drives
        from ground_truth import GRAPH_PATH, configure_ground_truth
        from scene import create_stage

        spec = json.loads((SIM_DIR.parent /
                           'ros2_ws/src/amr_description/config/compact_amr.json').read_text())
        stage = create_stage(True, SIM_DIR / 'omni_amr.usd', True)
        robot = stage.GetPrimAtPath('/World/compact_mecanum_amr')
        matrix = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 30))
        matrix.SetTranslateOnly(Gf.Vec3d(0.4, -0.2, 0))
        UsdGeom.Xformable(robot).MakeMatrixXform().Set(matrix)
        drives = configure_drives(stage, spec)
        configure_ground_truth(stage, spec)
        configure_ground_truth(stage, spec)
        app.update()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        checked = 0
        saw_offset_and_heading = False
        peak_error = 0.0
        for i in range(180):
            apply_command(drives, spec, [0.05, 0, 0] if i < 120 else [0, 0, 0])
            app.update()
            if i < 20:
                continue
            actual = omni.physx.get_physx_interface().get_rigidbody_transformation(
                '/World/compact_mecanum_amr/base_link')
            assert actual['ret_val'], actual
            position = og.Controller.attribute(GRAPH_PATH + '/Read.outputs:translation').get()
            rotation = og.Controller.attribute(GRAPH_PATH + '/Read.outputs:orientation').get()
            error = math.dist(position, actual['position'])
            dot = abs(sum(a*b for a, b in zip(rotation, actual['rotation'])))
            assert error < 0.01, (error, position, actual)
            assert dot > 0.999, (dot, rotation, actual)
            # A world pose must contain the spawn offset and heading.
            saw_offset_and_heading |= (math.hypot(position[0], position[1]) > 0.05
                                       and abs(rotation[2]) > 0.05)
            peak_error = max(peak_error, error)
            checked += 1
        timeline.stop()
        assert saw_offset_and_heading, 'The fixture never exercised a nonzero world pose'
        print(f'PASS ground truth: {checked} comparisons, max position error {peak_error:.6f} m; '
              'offset spawn and idempotency',
              flush=True)
    except Exception:
        import traceback
        traceback.print_exc()
        print('FAIL ground truth fixture', flush=True)
        # Kit fast shutdown can terminate before an exception reaches Python.
        # This isolated test must still return a nonzero failure status.
        import os
        os._exit(1)
    finally:
        app.close()


if __name__ == '__main__':
    main()
