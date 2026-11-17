# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Author a compact, asset-free mapping room and black robot visuals in USD.

Call prepare_mapping_scene(stage) before playback, then explicitly save the stage.
It edits the floor, test walls, robot root placement and visual bindings only.
"""

AREA_PATH = '/World/MappingArea'
ROBOT_PATH = '/World/compact_mecanum_amr'
START_POSITION = (-2.0, -1.0, 0.0)
# Name, centre XY, footprint XY. All walls are 1 m tall and 0.1 m thick.
WALLS = (
    ('North', (0.0, 2.0), (6.0, 0.1)),
    ('South', (0.0, -2.0), (6.0, 0.1)),
    ('East', (3.0, 0.0), (0.1, 4.0)),
    ('West', (-3.0, 0.0), (0.1, 4.0)),
    ('InnerVertical', (0.7, 0.5), (0.1, 1.4)),
    ('InnerHorizontal', (1.4, 1.2), (1.5, 0.1)),
    ('ShortPartition', (-1.0, 0.8), (0.8, 0.1)),
)


def _set_transform(prim, position, scale):
    """Replace only this scene object's transform stack, not robot joint/link poses."""
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    for attribute in list(prim.GetAttributes()):
        if attribute.GetName().startswith('xformOp:'):
            prim.RemoveProperty(attribute.GetName())
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(*scale))


def _material(stage, name, color):
    """Use the built-in USD shader; no MDL downloads or texture files are needed."""
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, '/World/Looks/' + name)
    shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + '/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return material


def prepare_mapping_scene(stage):
    """Build seven static walls, place the AMR, and make its visual meshes black.

    Keeps robot mass/inertia, joints, sensors, graphs and physics materials intact.
    Repeated calls reuse the same prims. Does not save or start simulation.
    """
    from pxr import UsdGeom, UsdPhysics, UsdShade

    robot = stage.GetPrimAtPath(ROBOT_PATH)
    floor = stage.GetPrimAtPath('/World/Ground')
    if not robot or not floor.IsA(UsdGeom.Cube):
        raise ValueError('Expected the compact AMR and Cube /World/Ground')
    # Replace the previously tried remote maze with these local primitive walls.
    if stage.GetPrimAtPath('/World/labyrinth_example'):
        stage.RemovePrim('/World/labyrinth_example')
    walls_material = _material(stage, 'MappingWalls', (0.72, 0.75, 0.78))
    floor_material = _material(stage, 'MappingFloor', (0.36, 0.38, 0.40))
    black = _material(stage, 'AMRBlack', (0.012, 0.012, 0.012))
    UsdGeom.Cube(floor).CreateSizeAttr(1.0)
    _set_transform(floor, (0.0, 0.0, -0.05), (6.4, 4.4, 0.1))
    UsdPhysics.CollisionAPI.Apply(floor)
    # This is an all-purpose visual binding; the separate physics binding remains.
    UsdShade.MaterialBindingAPI.Apply(floor).Bind(floor_material)
    UsdGeom.Xform.Define(stage, AREA_PATH)
    for name, (x, y), (sx, sy) in WALLS:
        wall = UsdGeom.Cube.Define(stage, AREA_PATH + '/' + name)
        wall.CreateSizeAttr(1.0)
        _set_transform(wall.GetPrim(), (x, y, 0.5), (sx, sy, 1.0))
        UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(wall.GetPrim()).Bind(walls_material)
    _set_transform(robot, START_POSITION, (1.0, 1.0, 1.0))
    count = 0
    for link in robot.GetChildren():
        visuals = link.GetChild('visuals')
        if visuals:
            UsdShade.MaterialBindingAPI.Apply(visuals).Bind(
                black, UsdShade.Tokens.strongerThanDescendants)
            count += 1
    if not count:
        raise ValueError('Robot has no visual groups to color')
    return count
