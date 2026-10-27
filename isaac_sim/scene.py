# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Editor extensions, stage, ground, lighting and physics."""


def create_stage(headless, scene_path=None, ros2=False):
    import omni.kit.app
    import omni.usd
    from pxr import UsdGeom

    manager = omni.kit.app.get_app().get_extension_manager()
    if not headless:
        manager.set_extension_enabled_immediate('omni.graph.window.action', True)
    manager.set_extension_enabled_immediate('isaacsim.asset.importer.urdf', True)
    if ros2:
        manager.set_extension_enabled_immediate('isaacsim.ros2.bridge', True)
        if not manager.is_extension_enabled('isaacsim.ros2.bridge'):
            raise RuntimeError('ROS 2 Bridge could not be enabled; check Isaac Sim ROS libraries')

    context = omni.usd.get_context()
    if scene_path is not None:
        if not context.open_stage(str(scene_path)):
            raise RuntimeError(f'Could not open scene: {scene_path}')
        return context.get_stage()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    return stage


def add_environment(stage):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

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
