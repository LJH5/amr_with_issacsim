#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""GPU fixture: a visible target changes from red/1.4 m to blue/1.9 m.

Receive with check_topics.py --camera-fixture. Never saves the user's USD.
"""

import json
from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        import omni.timeline
        from pxr import Gf, UsdGeom, UsdPhysics
        from camera import configure_camera
        from drive import configure_drives
        from feedback import configure_feedback
        from lidar import configure_lidars
        from scene import create_stage

        spec = json.loads((SIM_DIR.parent /
                           'ros2_ws/src/amr_description/config/compact_amr.json').read_text())
        stage = create_stage(True, SIM_DIR / 'omni_amr.usd', True)
        configure_drives(stage, spec)
        configure_feedback(stage, spec)
        configure_lidars(stage, spec)
        configure_camera(stage, spec)
        configure_camera(stage, spec)  # Repeated setup must reuse the same nodes/camera.
        mount = next(prim for prim in stage.Traverse() if prim.GetName() == 'camera_front_link')
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(mount)
        cube = UsdGeom.Cube.Define(stage, '/World/CameraTestTarget')
        cube.CreateSizeAttr(1.0)
        translation = cube.AddTranslateOp()
        cube.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
        cube.AddScaleOp().Set(Gf.Vec3f(0.2, 0.4, 0.4))
        translation.Set(matrix.Transform(Gf.Vec3d(1.5, 0, 0)))
        color = cube.CreateDisplayColorAttr([Gf.Vec3f(1, 0, 0)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        app.update()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        moved = False
        for _ in range(600):
            if not moved and timeline.get_current_time() > 4.0:
                translation.Set(matrix.Transform(Gf.Vec3d(2.0, 0, 0)))
                color.Set([Gf.Vec3f(0, 0, 1)])
                moved = True
                print('Camera fixture: target blue at 1.9 m surface distance', flush=True)
            app.update()
        timeline.stop()
        print('Camera fixture completed', flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
