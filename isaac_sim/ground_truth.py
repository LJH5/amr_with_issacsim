# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Publish the physics chassis world pose for measurement, without odom/TF."""

from feedback import find_feedback_prims

GRAPH_PATH = '/World/AMRGroundTruth'
TOPIC = '/ground_truth/pose'


def configure_ground_truth(stage, spec):
    import omni.graph.core as og
    from pxr import UsdGeom, UsdPhysics

    if not abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9:
        raise ValueError('Ground truth requires stage units in metres')
    _, imu = find_feedback_prims(stage, spec['wheels']['order'])
    base = stage.GetPrimAtPath(str(stage.GetPrimAtPath(imu).GetParent().GetPath()) + '/base_link')
    if not base or not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('Ground truth requires the base_link rigid body')
    graph = stage.GetPrimAtPath(GRAPH_PATH)
    if graph and graph.GetTypeName() != 'OmniGraph':
        raise RuntimeError('Ground truth graph path is occupied')
    if not graph:
        og.Controller.create_graph({'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'})
    og.get_graph_by_path(GRAPH_PATH).change_pipeline_stage(
        og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION)
    types = {'Tick': 'omni.graph.action.OnPlaybackTick',
             'Context': 'isaacsim.ros2.bridge.ROS2Context',
             'Time': 'isaacsim.core.nodes.IsaacReadSimulationTime',
             'Stamp': 'isaacsim.core.nodes.IsaacTimeSplitter',
             'Read': 'isaacsim.core.nodes.IsaacReadWorldPose',
             'Position': 'omni.graph.nodes.BreakVector3',
             'Orientation': 'omni.graph.nodes.BreakVector4',
             'Publish': 'isaacsim.ros2.bridge.ROS2Publisher'}
    for name, kind in types.items():
        path = GRAPH_PATH + '/' + name
        node = stage.GetPrimAtPath(path)
        if node:
            if node.GetAttribute('node:type').Get() != kind:
                raise RuntimeError('Unexpected ground truth node: ' + path)
        else:
            og.Controller.create_node(path, kind)

    def value(name, data):
        og.Controller.set(og.Controller.attribute(GRAPH_PATH + '/' + name), data)

    def connect(source, destination):
        source, destination = [GRAPH_PATH + '/' + p for p in (source, destination)]
        existing = [str(p) for p in stage.GetAttributeAtPath(destination).GetConnections()]
        if existing and existing != [source]:
            raise RuntimeError('Ground truth input already connected: ' + destination)
        if not existing:
            og.Controller.connect(og.Controller.attribute(source),
                                  og.Controller.attribute(destination))

    value('Context.inputs:useDomainIDEnvVar', True)
    value('Time.inputs:resetOnStop', False)
    value('Publish.inputs:messagePackage', 'geometry_msgs')
    value('Publish.inputs:messageSubfolder', 'msg')
    value('Publish.inputs:messageName', 'PoseStamped')
    value('Publish.inputs:topicName', TOPIC)
    value('Publish.inputs:header:frame_id', 'world')
    value('Publish.inputs:queueSize', 10)
    stage.GetPrimAtPath(GRAPH_PATH + '/Read').CreateRelationship(
        'inputs:prim').SetTargets([base.GetPath()])
    for source, destination in (
            ('Tick.outputs:tick', 'Publish.inputs:execIn'),
            ('Context.outputs:context', 'Publish.inputs:context'),
            ('Time.outputs:simulationTime', 'Stamp.inputs:time'),
            ('Stamp.outputs:seconds', 'Publish.inputs:header:stamp:sec'),
            ('Stamp.outputs:nanoseconds', 'Publish.inputs:header:stamp:nanosec'),
            ('Read.outputs:translation', 'Position.inputs:tuple'),
            ('Read.outputs:orientation', 'Orientation.inputs:tuple')):
        connect(source, destination)
    # ReadWorldPose emits world IJKR (xyzw), matching ROS quaternion components.
    for node, field, axes in (('Position', 'position', 'xyz'),
                              ('Orientation', 'orientation', 'xyzw')):
        for axis in axes:
            connect(f'{node}.outputs:{axis}', f'Publish.inputs:pose:{field}:{axis}')
    print(f'Physics ground truth: {base.GetPath()} -> {TOPIC} (world)', flush=True)
