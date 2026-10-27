#!/usr/bin/env python3
# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""USD-only regression test; requires Isaac Sim's USD libraries, not a GUI."""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from pxr import Sdf, Usd, UsdGeom


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot import import_robot


class GraphAnchorChecks(unittest.TestCase):
    def check_import(self, modern):
        stage = Usd.Stage.CreateInMemory()
        world = UsdGeom.Xform.Define(stage, '/World').GetPrim()
        stage.SetDefaultPrim(world)

        def import_nested_robot():
            UsdGeom.Xform.Define(stage, '/World/compact_mecanum_amr')
            # Reproduce the invalid metadata observed in amr_scene.usda.
            stage.SetMetadata('defaultPrim', 'compact_mecanum_amr')
            self.assertTrue(stage.HasDefaultPrim())
            self.assertFalse(stage.GetDefaultPrim().IsValid())

        names = ('omni', 'omni.kit', 'omni.kit.commands', 'isaacsim',
                 'isaacsim.asset', 'isaacsim.asset.importer', 'isaacsim.asset.importer.urdf')
        modules = {name: ModuleType(name) for name in names}
        for name, module in modules.items():
            if '.' in name:
                parent, child = name.rsplit('.', 1)
                setattr(modules[parent], child, module)
        urdf = modules['isaacsim.asset.importer.urdf']
        if modern:
            urdf.URDFImporterConfig = lambda **kwargs: SimpleNamespace(**kwargs)

            # Return a real referenceable layer so the production path is exercised.
            asset = Usd.Stage.CreateInMemory()
            asset.SetDefaultPrim(UsdGeom.Xform.Define(asset, '/Robot').GetPrim())

            def import_asset(config):
                import_nested_robot()
                return SimpleNamespace(import_urdf=lambda: asset.GetRootLayer().identifier)

            urdf.URDFImporter = import_asset
        else:
            urdf._urdf = SimpleNamespace(ImportConfig=SimpleNamespace)

            def execute(command, **kwargs):
                self.assertFalse(kwargs['import_config'].make_default_prim)
                import_nested_robot()
                return True, '/World/compact_mecanum_amr'

            modules['omni.kit.commands'].execute = execute

        with patch.dict(sys.modules, modules):
            import_robot(stage, Path('robot.urdf'), Path('/tmp'))
        self.assertEqual(stage.GetDefaultPrim().GetPath(), Sdf.Path('/World'))
        graph_path = Sdf.Path('ActionGraph').MakeAbsolutePath(stage.GetDefaultPrim().GetPath())
        self.assertEqual(graph_path, Sdf.Path('/World/ActionGraph'))
        self.assertTrue(stage.GetPrimAtPath('/World/compact_mecanum_amr').IsValid())

    def test_legacy_import_preserves_world_anchor(self):
        self.check_import(modern=False)

    def test_modern_import_preserves_world_anchor(self):
        self.check_import(modern=True)


if __name__ == '__main__':
    unittest.main()
