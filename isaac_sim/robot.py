# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Import the shared mobile-robot URDF using the installed Isaac Sim API."""


def import_robot(stage, source, build):
    import omni.kit.commands
    import isaacsim.asset.importer.urdf as urdf

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
        config.make_default_prim = False
        success, _ = omni.kit.commands.execute(
            'URDFParseAndImportFile', urdf_path=str(source), import_config=config)
        if not success:
            raise RuntimeError('URDF import failed')

    # The importer can author the nested robot name as a root-level defaultPrim.
    # Action Graph's editor resolves new graph paths relative to this prim.
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
