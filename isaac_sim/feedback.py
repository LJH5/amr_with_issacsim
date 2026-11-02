# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim 5.1 physics IMU and measured articulation feedback to ROS 2."""

import math


GRAPH_PATH = '/World/AMRFeedback'
IMU_NAME = 'ImuSensor'
IMU_FRAME = 'imu_link'
JOINT_TOPIC = '/joint_states'


def find_feedback_prims(stage, wheel_order):
    from pxr import UsdPhysics

    roots = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError(f'Feedback requires one robot articulation; found {len(roots)}')
    robot_path = roots[0].GetParent().GetPath()
    imu = stage.GetPrimAtPath(str(robot_path) + '/' + IMU_FRAME)
    if not imu or not imu.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f'IMU mounting link must be a rigid body: {robot_path}/{IMU_FRAME}')
    joints = {prim.GetName() for prim in stage.Traverse()
              if prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetPath().HasPrefix(robot_path)}
    missing = [name + '_wheel_joint' for name in wheel_order if name + '_wheel_joint' not in joints]
    if missing:
        raise RuntimeError(f'Missing wheel joints for feedback: {missing}')
    return str(roots[0].GetPath()), str(imu.GetPath())


def create_imu_sensor(stage, parent_path, rate_hz):
    import omni.kit.commands

    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError('IMU rate_hz must be positive and finite')
    path = parent_path + '/' + IMU_NAME
    sensor = stage.GetPrimAtPath(path)
    if sensor:
        if sensor.GetTypeName() != 'IsaacImuSensor':
            raise RuntimeError(f'IMU sensor path is occupied by a different prim: {path}')
    else:
        success, _ = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor', path='/' + IMU_NAME, parent=parent_path,
            sensor_period=1.0 / rate_hz,
            linear_acceleration_filter_size=1, angular_velocity_filter_size=1,
            orientation_filter_size=1)
        sensor = stage.GetPrimAtPath(path)
        if not success or not sensor:
            raise RuntimeError(f'Could not create physics IMU: {path}')
    sensor.GetAttribute('sensorPeriod').Set(1.0 / rate_hz)
    sensor.GetAttribute('enabled').Set(True)
    return path


def configure_feedback(stage, spec):
    import omni.graph.core as og
    from pxr import UsdGeom

    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError('Feedback expects stage units in metres')
    articulation_path, imu_link = find_feedback_prims(stage, spec['wheels']['order'])
    imu_spec = spec['sensors']['imu']
    imu_path = create_imu_sensor(stage, imu_link, imu_spec['rate_hz'])
    graph = stage.GetPrimAtPath(GRAPH_PATH)
    if graph:
        if graph.GetTypeName() != 'OmniGraph':
            raise RuntimeError(f'Feedback graph path is occupied: {GRAPH_PATH}')
    else:
        og.Controller.create_graph({'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'})
    graph_api = og.get_graph_by_path(GRAPH_PATH)
    simulation_stage = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
    if graph_api.get_pipeline_stage() != simulation_stage:
        graph_api.change_pipeline_stage(simulation_stage)

    node_types = {
        'Tick': 'omni.graph.action.OnPlaybackTick',
        'Context': 'isaacsim.ros2.bridge.ROS2Context',
        'SimTime': 'isaacsim.core.nodes.IsaacReadSimulationTime',
        'ReadIMU': 'isaacsim.sensors.physics.IsaacReadIMU',
        'PublishIMU': 'isaacsim.ros2.bridge.ROS2PublishImu',
        'PublishJoints': 'isaacsim.ros2.bridge.ROS2PublishJointState',
    }
    for name, node_type in node_types.items():
        path = GRAPH_PATH + '/' + name
        prim = stage.GetPrimAtPath(path)
        if prim:
            if prim.GetAttribute('node:type').Get() != node_type:
                raise RuntimeError(f'Unexpected node type at {path}')
        else:
            # Use an absolute node path; do not recreate an existing graph.
            og.Controller.create_node(path, node_type)

    connections = (
        ('Tick.outputs:tick', 'ReadIMU.inputs:execIn'),
        ('Tick.outputs:tick', 'PublishJoints.inputs:execIn'),
        ('ReadIMU.outputs:execOut', 'PublishIMU.inputs:execIn'),
        ('ReadIMU.outputs:linAcc', 'PublishIMU.inputs:linearAcceleration'),
        ('ReadIMU.outputs:angVel', 'PublishIMU.inputs:angularVelocity'),
        # Both are IJKR/xyzw graph quaternions: do not reorder here.
        ('ReadIMU.outputs:orientation', 'PublishIMU.inputs:orientation'),
        ('SimTime.outputs:simulationTime', 'PublishIMU.inputs:timeStamp'),
        ('SimTime.outputs:simulationTime', 'PublishJoints.inputs:timeStamp'),
        ('Context.outputs:context', 'PublishIMU.inputs:context'),
        ('Context.outputs:context', 'PublishJoints.inputs:context'),
    )
    for source, destination in connections:
        source_path = GRAPH_PATH + '/' + source
        destination_path = GRAPH_PATH + '/' + destination
        destination_attribute = stage.GetAttributeAtPath(destination_path)
        if not destination_attribute:
            raise RuntimeError(f'Feedback node input is unavailable: {destination_path}')
        existing = [str(path) for path in destination_attribute.GetConnections()]
        if existing and existing != [source_path]:
            raise RuntimeError(f'Feedback input already has another connection: {destination_path}')
        if not existing:
            og.Controller.connect(og.Controller.attribute(source_path),
                                  og.Controller.attribute(destination_path))

    values = {
        'Context.inputs:useDomainIDEnvVar': True,
        'SimTime.inputs:resetOnStop': False,
        'ReadIMU.inputs:readGravity': True,
        'ReadIMU.inputs:useLatestData': False,
        'PublishIMU.inputs:topicName': imu_spec['topic'],
        'PublishIMU.inputs:frameId': IMU_FRAME,
        'PublishIMU.inputs:publishAngularVelocity': True,
        'PublishIMU.inputs:publishLinearAcceleration': True,
        'PublishIMU.inputs:publishOrientation': True,
        'PublishJoints.inputs:topicName': JOINT_TOPIC,
    }
    for attribute, value in values.items():
        og.Controller.set(og.Controller.attribute(GRAPH_PATH + '/' + attribute), value)
    stage.GetPrimAtPath(GRAPH_PATH + '/ReadIMU').CreateRelationship(
        'inputs:imuPrim').SetTargets([imu_path])
    stage.GetPrimAtPath(GRAPH_PATH + '/PublishJoints').CreateRelationship(
        'inputs:targetPrim').SetTargets([articulation_path])
    print(f'Physics feedback: {imu_spec["topic"]} ({IMU_FRAME}), {JOINT_TOPIC}; graph {GRAPH_PATH}')
