# Copyright 2026 LJH5
# SPDX-License-Identifier: Apache-2.0
"""Scene assembly and simulation playback lifecycle."""

from drive import apply_command, configure_drives, ramp_command
from robot import import_robot
from scene import add_environment, create_stage


def run(app, args, spec, sim_dir, description):
    # Kit/USD imports stay inside functions called after SimulationApp starts.
    import omni.timeline

    stage = create_stage(args.headless)
    build = sim_dir / "build"
    build.mkdir(exist_ok=True)
    import_robot(stage, description / "urdf/compact_amr.urdf", build)
    add_environment(stage)
    drives = configure_drives(stage, spec)
    # Save at rest; launching this saved scene never starts a drive command.
    stage.GetRootLayer().Export(str(build / 'amr_scene.usda'))
    print(f'Scene saved: {build / "amr_scene.usda"}')
    print('Sensor mounts only: ROS bridge and live sensor outputs are not configured.')

    timeline = omni.timeline.get_timeline_interface()
    run_loop(app, timeline, args, spec, drives)


def run_loop(app, timeline, args, spec, drives):
    timeline.play()
    steps = 0
    last_time = 0.0
    previous = [0.0, 0.0, 0.0]
    try:
        while app.is_running() and (args.steps == 0 or steps < args.steps):
            now = timeline.get_current_time()
            dt = max(0.0, min(now - last_time, 0.1))
            last_time = now
            target = [args.vx, args.vy, args.wz] if now < args.drive_seconds else [0.0]*3
            previous = ramp_command(previous, target, dt, spec['motion_targets'])
            apply_command(drives, spec, previous)
            app.update()
            steps += 1
    finally:
        for drive in drives:
            drive.GetTargetVelocityAttr().Set(0.0)
        timeline.stop()
