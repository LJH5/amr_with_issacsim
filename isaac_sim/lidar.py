# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Two PhysX 2D lidars and their ROS 2 LaserScan/static TF graphs (Isaac Sim 5.1)."""

import math


GRAPH_PATH = '/World/AMRLidar'
LIDAR_NAMES = ('lidar_front_left', 'lidar_rear_right')
SENSOR_NAME = 'LidarSensor'
# Put the scan plane above the 40 mm tall mounting cylinder.
SCAN_HEIGHT_M = 0.025


def validate_lidar_spec(config):
    fov = config['horizontal_fov_deg']
    resolution = config['horizontal_resolution_deg']
    minimum, maximum = config['range_m']
    rate = config['rate_hz']
    if not all(math.isfinite(value) for value in (fov, resolution, minimum, maximum, rate)):
        raise ValueError('LiDAR parameters must be finite')
    if not (0 < resolution < fov <= 360 and 0 < minimum < maximum and rate > 0):
        raise ValueError('Invalid LiDAR FOV, resolution, range or rate')
    if not math.isclose(fov / resolution, round(fov / resolution)):
        raise ValueError('LiDAR FOV must be divisible by horizontal resolution')


def scan_metadata(config):
    validate_lidar_spec(config)
    fov, resolution = config['horizontal_fov_deg'], config['horizontal_resolution_deg']
    return {'angle_min': math.radians(-fov / 2),
            'angle_max': math.radians(fov / 2 - resolution),
            'angle_increment': math.radians(resolution),
            'time_increment': resolution / (360 * config['rate_hz']),
            'scan_time': 1 / config['rate_hz'],
            'range_min': config['range_m'][0], 'range_max': config['range_m'][1]}


def create_lidar(stage, parent_path, config):
    import omni.kit.commands
    from pxr import Gf, UsdGeom

    validate_lidar_spec(config)
    path = parent_path + '/' + SENSOR_NAME
    sensor = stage.GetPrimAtPath(path)
    if sensor:
        if sensor.GetTypeName() != 'Lidar':
            raise RuntimeError(f'LiDAR path is occupied by a different prim: {path}')
    else:
        success, _ = omni.kit.commands.execute(
            'RangeSensorCreateLidar', path='/' + SENSOR_NAME, parent=parent_path,
            translation=Gf.Vec3d(0, 0, SCAN_HEIGHT_M),
            min_range=config['range_m'][0], max_range=config['range_m'][1],
            horizontal_fov=config['horizontal_fov_deg'],
            horizontal_resolution=config['horizontal_resolution_deg'],
            # PhysX requires FOV > 0; this pair yields one horizontal row (zenith 0).
            vertical_fov=1.0, vertical_resolution=1.0,
            rotation_rate=config['rate_hz'], high_lod=False,
            draw_points=False, draw_lines=False, yaw_offset=0.0,
            enable_semantics=False)
        sensor = stage.GetPrimAtPath(path)
        if not success or not sensor:
            raise RuntimeError(f'Could not create PhysX LiDAR: {path}')
    values = {'minRange': config['range_m'][0], 'maxRange': config['range_m'][1],
              'horizontalFov': config['horizontal_fov_deg'],
              'horizontalResolution': config['horizontal_resolution_deg'],
              'verticalFov': 1.0, 'verticalResolution': 1.0,
              'rotationRate': config['rate_hz'], 'yawOffset': 0.0, 'enabled': True}
    for name, value in values.items():
        attribute = sensor.GetAttribute(name)
        if not attribute:
            raise RuntimeError(f'PhysX LiDAR attribute is unavailable: {path}.{name}')
        attribute.Set(value)
    # Keep a known transform when reopening a saved graph/sensor.
    UsdGeom.Xformable(sensor).ClearXformOpOrder()
    UsdGeom.Xformable(sensor).AddTranslateOp().Set(Gf.Vec3d(0, 0, SCAN_HEIGHT_M))
    return path


def configure_lidars(stage, spec):
    import omni.graph.core as og
    from pxr import UsdGeom, UsdPhysics

    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError('LiDAR expects stage units in metres')
    roots = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError('LiDAR requires exactly one robot articulation')
    robot = str(roots[0].GetParent().GetPath())
    base = stage.GetPrimAtPath(robot + '/base_link')
    if not base:
        raise RuntimeError('LiDAR TF requires base_link')
    sensors = {}
    for name in LIDAR_NAMES:
        parent = stage.GetPrimAtPath(robot + '/' + name + '_link')
        if not parent or not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f'Missing LiDAR rigid mounting link: {name}_link')
        sensors[name] = create_lidar(stage, str(parent.GetPath()), spec['sensors'][name])

    graph = stage.GetPrimAtPath(GRAPH_PATH)
    if graph:
        if graph.GetTypeName() != 'OmniGraph':
            raise RuntimeError(f'LiDAR graph path is occupied: {GRAPH_PATH}')
    else:
        og.Controller.create_graph({'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'})
    nodes = {'Tick': 'omni.graph.action.OnPlaybackTick',
             'Context': 'isaacsim.ros2.bridge.ROS2Context',
             'SimTime': 'isaacsim.core.nodes.IsaacReadSimulationTime',
             'Stamp': 'isaacsim.core.nodes.IsaacTimeSplitter'}
    for name in LIDAR_NAMES:
        nodes[name + '_Read'] = 'isaacsim.sensors.physx.IsaacReadLidarBeams'
        nodes[name + '_Publish'] = 'isaacsim.ros2.bridge.ROS2Publisher'
        nodes[name + '_TF'] = 'isaacsim.ros2.bridge.ROS2PublishRawTransformTree'
        nodes[name + '_Ready'] = 'omni.graph.nodes.Compare'
        nodes[name + '_Valid'] = 'omni.graph.action.Branch'
    for name, node_type in nodes.items():
        path = GRAPH_PATH + '/' + name
        node = stage.GetPrimAtPath(path)
        if node:
            if node.GetAttribute('node:type').Get() != node_type:
                raise RuntimeError(f'Unexpected LiDAR graph node type: {path}')
        else:
            og.Controller.create_node(path, node_type)

    def connect(source, destination):
        source_path = GRAPH_PATH + '/' + source
        destination_path = GRAPH_PATH + '/' + destination
        attribute = stage.GetAttributeAtPath(destination_path)
        if not attribute:
            raise RuntimeError(f'LiDAR graph input unavailable: {destination_path}')
        existing = [str(path) for path in attribute.GetConnections()]
        if existing and existing != [source_path]:
            raise RuntimeError(f'LiDAR input already connected elsewhere: {destination_path}')
        if not existing:
            og.Controller.connect(og.Controller.attribute(source_path),
                                  og.Controller.attribute(destination_path))

    def set_value(attribute, value):
        og.Controller.set(og.Controller.attribute(GRAPH_PATH + '/' + attribute), value)

    set_value('Context.inputs:useDomainIDEnvVar', True)
    set_value('SimTime.inputs:resetOnStop', False)
    connect('SimTime.outputs:simulationTime', 'Stamp.inputs:time')
    cache = UsdGeom.XformCache()
    base_inverse = cache.GetLocalToWorldTransform(base).GetInverse()
    for name, sensor_path in sensors.items():
        read, publish, tf = name + '_Read', name + '_Publish', name + '_TF'
        connect('Tick.outputs:tick', read + '.inputs:execIn')
        # The first reader execution has no data; wait for a populated scan.
        ready, valid = name + '_Ready', name + '_Valid'
        connect(read + '.outputs:numCols', ready + '.inputs:a')
        og.Controller.attribute(GRAPH_PATH + '/' + ready + '.inputs:b').set_resolved_type(
            og.Type(og.BaseDataType.INT))
        set_value(ready + '.inputs:b', 1)
        set_value(ready + '.inputs:operation', '>')
        connect(ready + '.outputs:result', valid + '.inputs:condition')
        connect(read + '.outputs:execOut', valid + '.inputs:execIn')
        connect(valid + '.outputs:execTrue', publish + '.inputs:execIn')
        # 5.1 ROS2PublishLaserScan computes time_increment before allocating its
        # first range buffer (division by zero). The standard generic publisher
        # avoids that SDK bug without changing or rebuilding the installed SDK.
        set_value(publish + '.inputs:messagePackage', 'sensor_msgs')
        set_value(publish + '.inputs:messageSubfolder', 'msg')
        set_value(publish + '.inputs:messageName', 'LaserScan')
        connect(read + '.outputs:linearDepthData', publish + '.inputs:ranges')
        connect('Stamp.outputs:seconds', publish + '.inputs:header:stamp:sec')
        connect('Stamp.outputs:nanoseconds', publish + '.inputs:header:stamp:nanosec')
        for field, value in scan_metadata(spec['sensors'][name]).items():
            set_value(publish + '.inputs:' + field, value)
        for target in (publish, tf):
            connect('Context.outputs:context', target + '.inputs:context')
        connect('SimTime.outputs:simulationTime', tf + '.inputs:timeStamp')
        connect('Tick.outputs:tick', tf + '.inputs:execIn')
        stage.GetPrimAtPath(GRAPH_PATH + '/' + read).CreateRelationship(
            'inputs:lidarPrim').SetTargets([sensor_path])
        frame = name + '_scan'
        set_value(publish + '.inputs:topicName', spec['sensors'][name]['topic'])
        set_value(publish + '.inputs:header:frame_id', frame)
        set_value(publish + '.inputs:queueSize', 5)
        # Unique scan children avoid competing with robot_state_publisher's link TFs.
        relative = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sensor_path)) * base_inverse
        quaternion = relative.ExtractRotationQuat()
        set_value(tf + '.inputs:topicName', '/tf_static')
        set_value(tf + '.inputs:staticPublisher', True)
        set_value(tf + '.inputs:parentFrameId', 'base_link')
        set_value(tf + '.inputs:childFrameId', frame)
        set_value(tf + '.inputs:translation', list(relative.ExtractTranslation()))
        set_value(tf + '.inputs:rotation', [*quaternion.GetImaginary(), quaternion.GetReal()])
        print(f'PhysX LiDAR: {sensor_path} -> {spec["sensors"][name]["topic"]} ({frame})')
