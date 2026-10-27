#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Import the shared URDF and run a small, standalone Isaac Sim test scene."""

import argparse
import json
import math
from pathlib import Path
import runpy

from mecanum import wheel_targets


SIM_DIR = Path(__file__).resolve().parent
DESCRIPTION = SIM_DIR.parent / 'ros2_ws/src/amr_description'


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--vx', type=float, default=0.0, help='Forward velocity, m/s')
    parser.add_argument('--vy', type=float, default=0.0, help='Leftward velocity, m/s')
    parser.add_argument('--wz', type=float, default=0.0, help='CCW yaw rate, rad/s')
    parser.add_argument('--drive-seconds', type=float, default=2.0,
                        help='Stop driving after this many simulated seconds')
    parser.add_argument('--steps', type=int, default=0,
                        help='Exit after N application updates; 0 runs until closed')
    args = parser.parse_args()
    if not math.isfinite(args.drive_seconds) or args.drive_seconds < 0 or args.steps < 0:
        parser.error('drive-seconds and steps must be nonnegative and finite')
    return args


def run(app, args, spec):
    # SimulationApp must exist before loading Kit/USD extensions.
    import omni.kit.app
    import omni.kit.commands
    import omni.timeline
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate('isaacsim.asset.importer.urdf', True)
    import isaacsim.asset.importer.urdf as urdf

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    build = SIM_DIR / 'build'
    build.mkdir(exist_ok=True)
    source = DESCRIPTION / 'urdf/compact_amr.urdf'
    if hasattr(urdf, 'URDFImporterConfig'):
        config = urdf.URDFImporterConfig(
            urdf_path=str(source), usd_path=str(build / 'robot'),
            fix_base=False, merge_fixed_joints=False, merge_mesh=False)
        asset = urdf.URDFImporter(config).import_urdf()
        if not asset:
            raise RuntimeError('URDF import did not return a USD asset')
        stage.DefinePrim('/World/AMR').GetReferences().AddReference(str(asset))
    else:
        config = urdf._urdf.ImportConfig()
        config.fix_base = False
        config.merge_fixed_joints = False
        config.self_collision = False
        config.import_inertia_tensor = True
        config.distance_scale = 1.0
        success, _ = omni.kit.commands.execute(
            'URDFParseAndImportFile', urdf_path=str(source), import_config=config)
        if not success:
            raise RuntimeError('URDF import failed')

    scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(120)
    ground = UsdGeom.Cube.Define(stage, '/World/Ground')
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    ground.AddScaleOp().Set(Gf.Vec3d(4, 4, 0.1))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.35, 0.35)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(1000)

    joints = {prim.GetName(): prim for prim in stage.Traverse()
              if prim.IsA(UsdPhysics.RevoluteJoint)}
    drives = []
    for name in spec['wheels']['order']:
        prim = joints[name + '_wheel_joint']
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(200.0)
        drive.CreateMaxForceAttr(spec['wheels']['joint_effort_limit_nm'])
        drive.CreateTargetVelocityAttr(0.0)
        drives.append(drive)
    for name, prim in joints.items():
        if '_roller_' in name:
            # Rollers must remain passive: remove importer-created servos.
            prim.RemoveAPI(UsdPhysics.DriveAPI, 'angular')
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(False)
    # Save at rest; launching this saved scene never starts a drive command.
    stage.GetRootLayer().Export(str(build / 'amr_scene.usda'))
    print(f'Scene saved: {build / "amr_scene.usda"}')
    print('Sensor mounts only: ROS bridge and live sensor outputs are not configured.')

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    steps = 0
    last_time = 0.0
    previous = [0.0, 0.0, 0.0]
    try:
        while app.is_running() and (args.steps == 0 or steps < args.steps):
            now = timeline.get_current_time()
            dt = max(0.0, min(now - last_time, 0.1))
            last_time = now
            target = [args.vx, args.vy, args.wz] if now < args.drive_seconds else [0.0]*3
            if target == [0.0]*3:
                previous = target  # Stop immediately when the command window expires.
            else:
                # Limit the planar acceleration as a vector, including diagonals.
                delta = [target[i] - previous[i] for i in (0, 1)]
                distance = math.hypot(*delta)
                limit = spec['motion_targets']['max_planar_acceleration_m_s2'] * dt
                scale = min(1.0, limit / distance) if distance else 1.0
                previous[:2] = [previous[i] + delta[i]*scale for i in (0, 1)]
                limit = spec['motion_targets']['max_yaw_acceleration_rad_s2'] * dt
                previous[2] += max(-limit, min(limit, target[2] - previous[2]))
            velocities = wheel_targets(spec, *previous)
            for drive, velocity in zip(drives, velocities):
                # USD angular drives use degrees/s; inverse kinematics uses rad/s.
                drive.GetTargetVelocityAttr().Set(math.degrees(velocity))
            app.update()
            steps += 1
    finally:
        for drive in drives:
            drive.GetTargetVelocityAttr().Set(0.0)
        timeline.stop()


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
        run(app, args, spec)
    finally:
        app.close()


if __name__ == '__main__':
    main()
