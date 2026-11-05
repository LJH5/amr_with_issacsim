#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Check actual USD roller geometry/materials; optionally export a repaired scene."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-scene', type=Path)
    parser.add_argument('--new-scene', action='store_true')
    args = parser.parse_args()
    if args.output_scene and args.output_scene.exists():
        parser.error('Output must be a new scene file')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from pxr import Gf, Usd, UsdPhysics, UsdShade
        from drive import configure_drives
        from materials import MATERIAL_PATHS, configure_contact_materials
        from robot import configure_roller_axes, configure_wheel_markers, import_robot
        from scene import add_environment, create_stage

        description = SIM_DIR.parent / 'ros2_ws/src/amr_description'
        spec = json.loads((description / 'config/compact_amr.json').read_text())
        stage = create_stage(True, None if args.new_scene else SIM_DIR / 'omni_amr.usd', True)
        if args.new_scene:
            import_robot(stage, description / 'urdf/compact_amr.urdf', SIM_DIR / 'build')
            add_environment(stage)
        before = sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse())
        for _ in range(2):
            configure_roller_axes(stage, description / 'urdf/compact_amr.urdf')
            configure_wheel_markers(stage, spec)
            configure_contact_materials(stage, spec)
            drives = configure_drives(stage, spec)
        assert len(drives) == 4
        assert sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()) == before
        assert sum(p.GetName() == 'RotationMarker' for p in stage.Traverse()) == 4
        joints = {p.GetName(): p for p in stage.Traverse()
                  if p.IsA(UsdPhysics.RevoluteJoint)}
        for name, hand in zip(spec['wheels']['order'], spec['wheels']['roller_handedness']):
            wheel = UsdPhysics.RevoluteJoint(joints[name + '_wheel_joint'])
            roller = UsdPhysics.RevoluteJoint(joints[name + '_roller_2_joint'])
            axis = Gf.Rotation(wheel.GetLocalRot0Attr().Get()).TransformDir(
                Gf.Rotation(roller.GetLocalRot0Attr().Get()).TransformDir(Gf.Vec3d(0, 0, 1)))
            assert math.isclose(axis[1] / axis[0], hand, abs_tol=1e-6), (name, axis)
        rollers = [p for p in joints.values() if '_roller_' in p.GetName()]
        assert len(rollers) == 32
        assert not any(p.HasAPI(UsdPhysics.DriveAPI, 'angular') for p in rollers)
        for label, path in MATERIAL_PATHS.items():
            material = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(path))
            assert math.isclose(material.GetStaticFrictionAttr().Get(),
                                spec['contact_materials'][label]['static_friction'], abs_tol=1e-6)
            assert math.isclose(material.GetDynamicFrictionAttr().Get(),
                                spec['contact_materials'][label]['dynamic_friction'], abs_tol=1e-6)
            assert math.isclose(material.GetRestitutionAttr().Get(),
                                spec['contact_materials'][label]['restitution'], abs_tol=1e-6)
        bound = 0
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.CollisionAPI) and '_roller_' in path:
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
                assert str(material.GetPath()) == MATERIAL_PATHS['rollers'], path
                bound += 1
        assert bound >= 32, bound
        material, _ = UsdShade.MaterialBindingAPI(
            stage.GetPrimAtPath('/World/Ground')).ComputeBoundMaterial('physics')
        assert str(material.GetPath()) == MATERIAL_PATHS['ground']
        invalid = deepcopy(spec)
        invalid['contact_materials']['ground']['dynamic_friction'] = -1
        try:
            configure_contact_materials(stage, invalid)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid contact settings accepted')
        if args.output_scene:
            args.output_scene.parent.mkdir(parents=True, exist_ok=True)
            assert stage.GetRootLayer().Export(str(args.output_scene))
            saved = Usd.Stage.Open(str(args.output_scene))
            assert saved.GetPrimAtPath(MATERIAL_PATHS['ground'])
            print('EXPORTED', args.output_scene, flush=True)
        print(f'PASS model fix: 32 roller frames, 4 markers, {bound} material bindings, '
              'floor material, passive rollers, idempotency', flush=True)
    except Exception:
        import os
        import traceback
        traceback.print_exc()
        print('FAIL model fix fixture', flush=True)
        os._exit(1)
    finally:
        app.close()


if __name__ == '__main__':
    main()
