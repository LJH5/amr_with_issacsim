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


def configure_roller_axes(stage, urdf_path):
    """Match saved/imported roller joint frames and link poses to canonical URDF.

    Only roller rest frames change. The chassis, wheel hubs, sensors and user's
    scene stay in place. Passive roller spin is reset at startup, before play.
    """
    import math
    import xml.etree.ElementTree as ET
    from pxr import Gf, UsdGeom, UsdPhysics

    descriptions = [j for j in ET.parse(urdf_path).getroot().findall('joint')
                    if '_roller_' in j.attrib['name']]
    if not descriptions:
        raise ValueError('URDF has no roller joints')
    joints = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.setdefault(prim.GetName(), []).append(prim)
    prepared = []
    for item in descriptions:
        name = item.attrib['name']
        matches = joints.get(name, [])
        if len(matches) != 1:
            raise RuntimeError('Exactly one roller joint required: ' + name)
        joint = UsdPhysics.RevoluteJoint(matches[0])
        parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if len(parents) != 1 or len(children) != 1:
            raise RuntimeError('Invalid roller joint bodies: ' + name)
        parent, child = (stage.GetPrimAtPath(p) for p in (parents[0], children[0]))
        if (parent.GetName() != item.find('parent').attrib['link']
                or child.GetName() != item.find('child').attrib['link']
                or not child.HasAPI(UsdPhysics.RigidBodyAPI)):
            raise RuntimeError('Roller bodies do not match URDF: ' + name)
        origin = item.find('origin')
        xyz = [float(v) for v in origin.attrib['xyz'].split()]
        roll, pitch, yaw = [float(v) for v in origin.attrib['rpy'].split()]
        cr, cp, cy = [math.cos(v/2) for v in (roll, pitch, yaw)]
        sr, sp, sy = [math.sin(v/2) for v in (roll, pitch, yaw)]
        quaternion = Gf.Quatd(cr*cp*cy + sr*sp*sy, Gf.Vec3d(
            sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy))
        prepared.append((joint, parent, child, xyz, quaternion))
    cache = UsdGeom.XformCache()
    for joint, parent, child, xyz, quaternion in prepared:
        local = Gf.Matrix4d(1).SetRotate(Gf.Rotation(quaternion))
        local.SetTranslateOnly(Gf.Vec3d(*xyz))
        world = local * cache.GetLocalToWorldTransform(parent)
        child_local = world * cache.GetLocalToWorldTransform(child.GetParent()).GetInverse()
        # PhysX writes rigid-body orientation as quatf during playback. Use the
        # native translate/orient/scale stack rather than a matrix-only stack,
        # which can make its writeback create an incompatible quatd attribute.
        xform = UsdGeom.Xformable(child)
        xform.ClearXformOpOrder()
        for attribute in list(child.GetAttributes()):
            if attribute.GetName().startswith('xformOp:'):
                child.RemoveProperty(attribute.GetName())
        transform = Gf.Transform(child_local)
        xform.AddTranslateOp().Set(transform.GetTranslation())
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Quatf(transform.GetRotation().GetQuat()))
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(transform.GetScale()))
        joint.CreateAxisAttr('Z')
        joint.CreateLocalPos0Attr(Gf.Vec3f(*xyz))
        joint.CreateLocalRot0Attr(Gf.Quatf(quaternion))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1))
    print(f'Roller frames: {len(prepared)} joints aligned with canonical URDF', flush=True)


def configure_wheel_markers(stage, spec):
    """Add visual-only orange hub marks so joint rotation is observable."""
    from pxr import Gf, UsdGeom, UsdPhysics

    wheels = spec['wheels']
    expected = {name + '_wheel' for name in wheels['order']}
    hubs = [prim for prim in stage.Traverse()
            if prim.GetName() in expected and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(hubs) != 4:
        raise RuntimeError('Four wheel hubs required for rotation markers')
    for hub in hubs:
        path = str(hub.GetPath()) + '/RotationMarker'
        existing = stage.GetPrimAtPath(path)
        if existing and not existing.IsA(UsdGeom.Cube):
            raise RuntimeError('Wheel marker path occupied: ' + path)
        marker = UsdGeom.Cube.Define(stage, path)
        marker.CreateSizeAttr(1)
        marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.65, 0.05)])
        side = 1 if 'left' in hub.GetName() else -1
        transform = UsdGeom.XformCommonAPI(marker.GetPrim())
        marker_position = Gf.Vec3d(
            wheels['hub_radius_m']*0.45, 0, side*(wheels['hub_width_m']/2 + 0.001))
        transform.SetTranslate(marker_position)
        transform.SetScale(Gf.Vec3f(wheels['hub_radius_m']*0.8, 0.006, 0.002))
