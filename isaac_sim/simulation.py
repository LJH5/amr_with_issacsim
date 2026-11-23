# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Scene assembly and simulation playback lifecycle."""

from drive import apply_command, apply_wheel_velocities, configure_drives, ramp_command
from camera import configure_camera
from feedback import configure_feedback
from ground_truth import configure_ground_truth
from lidar import configure_lidars
from robot import configure_roller_axes, configure_wheel_markers, import_robot
from materials import configure_contact_materials
from ros_commands import GraphWheelSubscriber
from scene import add_environment, create_stage


def run(app, args, spec, sim_dir, description):
    # Kit/USD imports stay inside functions called after SimulationApp starts.
    import omni.timeline
    import omni.usd

    stage = create_stage(args.headless, args.scene, args.ros2)
    if args.scene is None:
        build = sim_dir / "build"
        build.mkdir(exist_ok=True)
        import_robot(stage, description / "urdf/compact_amr.urdf", build)
        add_environment(stage)
    configure_roller_axes(stage, description / 'urdf/compact_amr.urdf')
    configure_wheel_markers(stage, spec)
    configure_contact_materials(stage, spec)
    drives = configure_drives(stage, spec)
    command_source = None
    if args.ros2:
        configure_feedback(stage, spec)
        configure_ground_truth(stage, spec)
        configure_lidars(stage, spec)
        configure_camera(stage, spec)
        if args.control == 'ros':
            command_source = GraphWheelSubscriber(stage, spec)
        # Process graph/schema changes before physics warm-start evaluates nodes.
        app.update()
    # Save at rest; launching this saved scene never starts a drive command.
    if args.scene is None:
        stage.GetRootLayer().Export(str(build / 'amr_scene.usda'))
        print(f'Scene saved: {build / "amr_scene.usda"}')
    else:
        print(f'Scene loaded: {args.scene} (save edits with Ctrl+S)')
    print(f'Wheel control source: {args.control}')

    timeline = omni.timeline.get_timeline_interface()
    run_loop(app, timeline, args, spec, drives, omni.usd.get_context().get_stage, command_source)


def run_loop(app, timeline, args, spec, drives, get_stage=None, command_source=None):
    initial_stage = get_stage() if get_stage is not None else None
    timeline.play()
    steps = 0
    last_time = 0.0
    last_command_time = 0.0
    previous = [0.0, 0.0, 0.0]
    try:
        while app.is_running() and (args.steps == 0 or steps < args.steps):
            if drives and ((get_stage is not None and get_stage() != initial_stage)
                           or not all(drives)):
                # File > Open/New can invalidate every cached USD drive handle.
                # Leave playback/UI running, but never control the replacement scene.
                drives = []
                print('Scene changed: CLI wheel control released; UI remains active.')
            now = timeline.get_current_time()
            dt = max(0.0, min(now - last_time, 0.1))
            last_time = now
            if command_source is not None:
                if not drives:
                    velocities = [0.0] * 4
                elif not timeline.is_playing() or now < last_command_time:
                    command_source.reset()
                    velocities = [0.0] * 4
                else:
                    velocities = command_source.poll()
                apply_wheel_velocities(drives, velocities)
            else:
                target = [args.vx, args.vy, args.wz] if now < args.drive_seconds else [0.0]*3
                previous = ramp_command(previous, target, dt, spec['motion_targets'])
                apply_command(drives, spec, previous)
            last_command_time = now
            app.update()
            steps += 1
    finally:
        same_stage = get_stage is None or get_stage() == initial_stage
        for drive in drives if same_stage else []:
            if drive:
                drive.GetTargetVelocityAttr().Set(0.0)
        timeline.stop()
