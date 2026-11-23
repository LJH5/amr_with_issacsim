# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Bind explicit contact materials to roller colliders and the test floor."""

import math

MATERIAL_PATHS = {'rollers': '/World/PhysicsMaterials/UrethaneRollers',
                  'ground': '/World/PhysicsMaterials/EpoxyGround'}


def configure_contact_materials(stage, spec):
    """Apply the shared dimensionless friction coefficients to actual colliders.

    Values are simulation starting parameters, not measured material constants.
    Existing unrelated materials and robot/scene geometry are preserved.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

    values = spec['contact_materials']
    for label in MATERIAL_PATHS:
        item = values[label]
        numbers = [item[key] for key in ('static_friction', 'dynamic_friction', 'restitution')]
        if (not all(math.isfinite(v) and v >= 0 for v in numbers)
                or numbers[1] > numbers[0] or numbers[2] > 1
                or item['friction_combine_mode'] not in ('average', 'min', 'multiply', 'max')):
            raise ValueError('Invalid contact material: ' + label)
        prim = stage.GetPrimAtPath(MATERIAL_PATHS[label])
        if prim and not prim.IsA(UsdShade.Material):
            raise RuntimeError('Physics material path occupied: ' + MATERIAL_PATHS[label])
    expected = {f'{name}_roller_{i}' for name in spec['wheels']['order']
                for i in range(spec['wheels']['rollers_per_wheel'])}
    rollers, found = [], set()
    # Author the source collider prims, not uneditable instance proxies. Bindings
    # on /colliders templates propagate to the imported robot instances.
    for prim in stage.Traverse():
        names = set(str(prim.GetPath()).split('/')) & expected
        if names and prim.HasAPI(UsdPhysics.CollisionAPI):
            rollers.append(prim)
            found.update(names)
    if found != expected:
        raise RuntimeError('Missing roller colliders: ' + ', '.join(sorted(expected - found)))
    floor = stage.GetPrimAtPath('/World/Ground')
    grounds = ([p for p in Usd.PrimRange(floor) if p.HasAPI(UsdPhysics.CollisionAPI)]
               if floor else [])
    if not grounds:
        raise RuntimeError('Contact materials require colliders under /World/Ground')
    for label, colliders in (('rollers', rollers), ('ground', grounds)):
        material = UsdShade.Material.Define(stage, MATERIAL_PATHS[label])
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(values[label]['static_friction'])
        physics.CreateDynamicFrictionAttr(values[label]['dynamic_friction'])
        physics.CreateRestitutionAttr(values[label]['restitution'])
        PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(
            values[label]['friction_combine_mode'])
        for prim in colliders:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, UsdShade.Tokens.strongerThanDescendants, 'physics')
    print(f'Contact materials: {len(rollers)} roller colliders, {len(grounds)} floor colliders; '
          'coefficients from compact_amr.json', flush=True)
