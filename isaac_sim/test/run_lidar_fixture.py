#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""GPU/DDS integration fixture: saved AMR and two moving collision targets.

Run with Isaac Sim Python; receive with check_topics.py --fixture.
The user's USD file is opened in memory and never saved.
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
        from drive import configure_drives
        from feedback import configure_feedback
        from camera import configure_camera
        from lidar import configure_lidars, LIDAR_NAMES, SENSOR_NAME
        from scene import create_stage

        spec = json.loads((SIM_DIR.parent /
                           'ros2_ws/src/amr_description/config/compact_amr.json').read_text())
        stage = create_stage(True, SIM_DIR / 'omni_amr.usd', True)
        configure_drives(stage, spec)
        configure_feedback(stage, spec)
        configure_camera(stage, spec)
        configure_lidars(stage, spec)
        # A repeated setup must reuse all graph nodes and sensors.
        configure_lidars(stage, spec)
        cache = UsdGeom.XformCache()
        targets = []
        for name in LIDAR_NAMES:
            sensor = next(prim for prim in stage.Traverse()
                          if prim.GetName() == SENSOR_NAME and prim.GetParent().GetName() == name + '_link')
            matrix = cache.GetLocalToWorldTransform(sensor)
            cube = UsdGeom.Cube.Define(stage, '/World/Test_' + name)
            cube.CreateSizeAttr(1.0)
            translation = cube.AddTranslateOp()
            cube.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotationQuat()))
            cube.AddScaleOp().Set(Gf.Vec3f(0.2, 0.3, 0.4))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            translation.Set(matrix.Transform(Gf.Vec3d(1.2, 0, 0)))
            targets.append((translation, matrix))
        app.update()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        moved = False
        for _ in range(720):
            if not moved and timeline.get_current_time() > 4.0:
                for translation, matrix in targets:
                    translation.Set(matrix.Transform(Gf.Vec3d(1.7, 0, 0)))
                moved = True
                print('LiDAR fixture: targets moved from 1.1 to 1.6 m surface distance', flush=True)
            app.update()
        from isaacsim.sensors.physx import _range_sensor
        interface = _range_sensor.acquire_lidar_sensor_interface()
        for prim in stage.Traverse():
            if prim.GetName() == SENSOR_NAME:
                path = str(prim.GetPath())
                assert interface.get_num_rows(path) == 1, 'Expected one scan row'
                assert all(abs(value) < 1e-6 for value in interface.get_zenith_data(path)), \
                    'Expected a horizontal scan plane'
                print('LIDAR_GEOMETRY', path, 'rows', interface.get_num_rows(path),
                      'zenith', interface.get_zenith_data(path), flush=True)
        timeline.stop()
        print('LiDAR fixture completed', flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
