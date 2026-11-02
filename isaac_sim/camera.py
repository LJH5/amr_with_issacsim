# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Front pinhole RGB-D camera and ROS 2 render pipelines for Isaac Sim 5.1."""

import math

GRAPH_PATH = '/World/AMRCamera'
OPTICAL_FRAME = 'camera_front_rgbd_optical_frame'


def camera_parameters(config, render_hz):
    width, height = config['resolution_px']
    fov, rate = config['horizontal_fov_deg'], config['rate_hz']
    near, far = config['depth_range_m']
    if not all(math.isfinite(value) for value in (width, height, fov, rate, near, far, render_hz)):
        raise ValueError('Camera parameters must be finite')
    if (not all(isinstance(value, int) and value > 0 for value in (width, height))
            or not 0 < fov < 180 or not 0 < near < far or not 0 < rate <= render_hz):
        raise ValueError('Invalid camera resolution, FOV, clipping range or rate')
    if not math.isclose(render_hz / rate, round(render_hz / rate)):
        raise ValueError('Camera rate must divide the stage render cadence')
    # USD camera focal length/aperture use the same units; square pixels.
    aperture = 20.955
    return {'horizontal_aperture': aperture, 'vertical_aperture': aperture * height / width,
            'focal_length': aperture / (2 * math.tan(math.radians(fov) / 2)),
            'frame_skip': round(render_hz / rate) - 1}


def configure_camera(stage, spec):
    import omni.graph.core as og
    from pxr import Gf, UsdGeom, UsdPhysics

    config = spec['sensors']['camera_front']
    parameters = camera_parameters(config, stage.GetTimeCodesPerSecond())
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError('Camera expects stage units in metres')
    roots = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError('Camera requires exactly one robot articulation')
    robot = str(roots[0].GetParent().GetPath())
    mount = stage.GetPrimAtPath(robot + '/camera_front_link')
    base = stage.GetPrimAtPath(robot + '/base_link')
    if not mount or not base:
        raise RuntimeError('Camera requires camera_front_link and base_link')
    path = str(mount.GetPath()) + '/RgbdCamera'
    existing = stage.GetPrimAtPath(path)
    if existing and not existing.IsA(UsdGeom.Camera):
        raise RuntimeError(f'Camera path is occupied: {path}')
    camera = UsdGeom.Camera.Define(stage, path)
    transform = UsdGeom.Xformable(camera)
    transform.ClearXformOpOrder()
    # USD: -Z forward/+Y up. Robot: +X forward/+Z up.
    transform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(0.5, Gf.Vec3d(0.5, -0.5, -0.5)))
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.CreateHorizontalApertureAttr().Set(parameters['horizontal_aperture'])
    camera.CreateVerticalApertureAttr().Set(parameters['vertical_aperture'])
    camera.CreateFocalLengthAttr().Set(parameters['focal_length'])
    camera.CreateHorizontalApertureOffsetAttr().Set(0)
    camera.CreateVerticalApertureOffsetAttr().Set(0)
    camera.CreateFStopAttr().Set(0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(*config['depth_range_m']))

    graph = stage.GetPrimAtPath(GRAPH_PATH)
    if graph and graph.GetTypeName() != 'OmniGraph':
        raise RuntimeError(f'Camera graph path is occupied: {GRAPH_PATH}')
    if not graph:
        og.Controller.create_graph({'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'})
    types = {'Tick': 'omni.graph.action.OnPlaybackTick',
             'Context': 'isaacsim.ros2.bridge.ROS2Context',
             'SimTime': 'isaacsim.core.nodes.IsaacReadSimulationTime',
             'Render': 'isaacsim.core.nodes.IsaacCreateRenderProduct',
             'Color': 'isaacsim.ros2.bridge.ROS2CameraHelper',
             'Depth': 'isaacsim.ros2.bridge.ROS2CameraHelper',
             'ColorInfo': 'isaacsim.ros2.bridge.ROS2CameraInfoHelper',
             'DepthInfo': 'isaacsim.ros2.bridge.ROS2CameraInfoHelper',
             'TF': 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'}
    for name, node_type in types.items():
        node_path = GRAPH_PATH + '/' + name
        node = stage.GetPrimAtPath(node_path)
        if node:
            if node.GetAttribute('node:type').Get() != node_type:
                raise RuntimeError(f'Unexpected camera node type: {node_path}')
        else:
            og.Controller.create_node(node_path, node_type)

    def connect(source, destination):
        source, destination = (GRAPH_PATH + '/' + value for value in (source, destination))
        attribute = stage.GetAttributeAtPath(destination)
        if not attribute:
            raise RuntimeError(f'Camera graph input unavailable: {destination}')
        existing = [str(value) for value in attribute.GetConnections()]
        if existing and existing != [source]:
            raise RuntimeError(f'Camera input already connected elsewhere: {destination}')
        if not existing:
            og.Controller.connect(og.Controller.attribute(source), og.Controller.attribute(destination))

    def set_value(attribute, value):
        og.Controller.set(og.Controller.attribute(GRAPH_PATH + '/' + attribute), value)

    connect('Tick.outputs:tick', 'Render.inputs:execIn')
    set_value('Context.inputs:useDomainIDEnvVar', True)
    set_value('SimTime.inputs:resetOnStop', False)
    set_value('Render.inputs:width', config['resolution_px'][0])
    set_value('Render.inputs:height', config['resolution_px'][1])
    stage.GetPrimAtPath(GRAPH_PATH + '/Render').CreateRelationship(
        'inputs:cameraPrim').SetTargets([path])
    for name, topic in (('Color', 'color_topic'), ('Depth', 'depth_topic'),
                        ('ColorInfo', 'color_info_topic'), ('DepthInfo', 'depth_info_topic')):
        connect('Render.outputs:execOut', name + '.inputs:execIn')
        connect('Render.outputs:renderProductPath', name + '.inputs:renderProductPath')
        connect('Context.outputs:context', name + '.inputs:context')
        for field, value in {'topicName': config[topic], 'frameId': OPTICAL_FRAME,
                             'frameSkipCount': parameters['frame_skip'],
                             'useSystemTime': False, 'resetSimulationTimeOnStop': False,
                             'queueSize': 5}.items():
            set_value(name + '.inputs:' + field, value)
    set_value('Color.inputs:type', 'rgb')
    set_value('Depth.inputs:type', 'depth')
    connect('Tick.outputs:tick', 'TF.inputs:execIn')
    connect('Context.outputs:context', 'TF.inputs:context')
    connect('SimTime.outputs:simulationTime', 'TF.inputs:timeStamp')
    cache = UsdGeom.XformCache()
    # ROS optical axes: +Z forward, +X right, +Y down (USD camera rotated X 180°).
    optical = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
    relative = (optical * cache.GetLocalToWorldTransform(camera.GetPrim())
                * cache.GetLocalToWorldTransform(base).GetInverse())
    quaternion = relative.ExtractRotationQuat()
    for field, value in {'topicName': '/tf_static', 'staticPublisher': True,
                         'parentFrameId': 'base_link', 'childFrameId': OPTICAL_FRAME,
                         'translation': list(relative.ExtractTranslation()),
                         'rotation': [*quaternion.GetImaginary(), quaternion.GetReal()]}.items():
        set_value('TF.inputs:' + field, value)
    print(f'RGB-D camera: {path}, {config["resolution_px"]}, target {config["rate_hz"]} Hz')
