#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""USD-only room checks; run with the Isaac Sim USD Python environment."""
from pathlib import Path
import sys

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
from mapping_scene import AREA_PATH, ROBOT_PATH, START_POSITION, WALLS, prepare_mapping_scene


def check(stage):
    """Verify light geometry, ground height, safe starting clearance and black bindings."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
    ground = stage.GetPrimAtPath('/World/Ground')
    bounds = cache.ComputeWorldBound(ground).ComputeAlignedRange()
    assert abs(bounds.GetMax()[2]) < 1e-6, bounds
    assert not stage.GetPrimAtPath('/World/labyrinth_example')
    assert len(stage.GetPrimAtPath(AREA_PATH).GetChildren()) == len(WALLS) == 7
    for name, (x, y), (sx, sy) in WALLS:
        wall = stage.GetPrimAtPath(AREA_PATH + '/' + name)
        assert wall.IsA(UsdGeom.Cube) and wall.HasAPI(UsdPhysics.CollisionAPI)
        assert not wall.HasAPI(UsdPhysics.RigidBodyAPI), 'Wall must be static'
        assert not wall.HasPayload() and not wall.HasAuthoredReferences()
        # Reserve a 0.7 m square around the robot's start for body/sensor clearance.
        assert (abs(START_POSITION[0] - x) > sx / 2 + 0.35
                or abs(START_POSITION[1] - y) > sy / 2 + 0.35)
    root = stage.GetPrimAtPath(ROBOT_PATH)
    assert root.GetAttribute('xformOp:translate').Get() == Gf.Vec3d(*START_POSITION)
    for link in root.GetChildren():
        visuals = link.GetChild('visuals')
        if visuals:
            material, _ = UsdShade.MaterialBindingAPI(visuals).ComputeBoundMaterial()
            assert str(material.GetPath()) == '/World/Looks/AMRBlack'
    shader = UsdShade.Shader(stage.GetPrimAtPath('/World/Looks/AMRBlack/Shader'))
    assert all(v < 0.02 for v in shader.GetInput('diffuseColor').Get())


if __name__ == '__main__':
    source = SIM / 'omni_amr.usd'
    stage = Usd.Stage.Open(str(source))
    base = stage.GetPrimAtPath(ROBOT_PATH + '/base_link')
    before = (base.GetAttribute('physics:mass').Get(),
              base.GetAttribute('physics:diagonalInertia').Get())
    for _ in range(2):
        prepare_mapping_scene(stage)
        check(stage)
    assert before == (base.GetAttribute('physics:mass').Get(),
                      base.GetAttribute('physics:diagonalInertia').Get())
    print('PASS compact room: 7 static walls, clear spawn, floor z=0, black AMR, '
          'mass/inertia unchanged and repeatable setup')
