"""MotionCore's standalone, observable RL simulation process.

Adapted from the uploaded Unitree deployment; see rl_assets/LICENSE.
Changes: CPU inference, live commands, atomic telemetry, measured speed,
Euler orientation, controlled stop, heartbeat and explicit failure reporting.
"""
from __future__ import annotations
import argparse
from contextlib import nullcontext
import math
from pathlib import Path
import sys
import time
import traceback

# Works from any current directory, including a path containing spaces.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend.rl_ipc import publish_json, read_json, sharing_error


def run(args):
    session = Path(args.session_dir)
    session.mkdir(parents=True, exist_ok=True)
    output = session/'telemetry.json'
    sequence = 0
    command = read_json(session/'command.json')
    token = command['session']
    state = 'STARTING'
    telemetry = dict(available=False, fallen=False, height=0., roll=0., pitch=0.,
                     leftFoot=False, rightFoot=False, source='unitree_rl')

    def publish(message=''):
        written = publish_json(output, {**telemetry, 'status': state, 'message': message,
                             'session': token, 'commandSeq': sequence, 'timestamp': time.time()})
        if not written:
            print('Telemetry file temporarily locked; retrying next snapshot.', flush=True)
        return written

    publish('Loading G1 policy')
    try:
        from backend.rl_policy import G1Policy
        robot = G1Policy()
        telemetry = robot.telemetry()
        telemetry['available'] = True
        state = 'WALKING'
        publish('RL policy ready')
        viewer_context = nullcontext(None)
        if not args.headless:
            import mujoco
            import mujoco.viewer
            viewer_context = mujoco.viewer.launch_passive(robot.model, robot.data)
        with viewer_context as viewer:
            if viewer is not None:
                import mujoco
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = robot.model.body('pelvis').id
                viewer.cam.distance = 2.4
                viewer.cam.elevation = -18
            stopping_at = None
            stable_for = 0.
            next_tick = time.perf_counter()
            next_read = next_tick
            next_publish = next_tick
            next_viewer = next_tick
            requested = 0.
            last_heartbeat = float(command['heartbeat'])
            while True:
                now = time.perf_counter()
                if now >= next_read:
                    try:
                        current = read_json(session/'command.json')
                        command = current
                    except OSError as exc:
                        if not sharing_error(exc):
                            raise
                        # Retain the last command and its original heartbeat.
                        # A persistent lock therefore still triggers deceleration.
                        current = command
                    if current.get('session') != token:
                        raise RuntimeError('RL command session changed unexpectedly.')
                    last_heartbeat = float(current['heartbeat'])
                    sequence = int(current['seq'])
                    requested = float(current['speed'])
                    if not math.isfinite(requested) or not 0 <= requested <= 1:
                        raise ValueError('Invalid RL speed command')
                    if current.get('stop') and stopping_at is None:
                        stopping_at = float(robot.data.time)
                    next_read = now + .05
                if time.time()-last_heartbeat > 5:
                    # Never keep walking indefinitely if the parent disappears.
                    stopping_at = stopping_at if stopping_at is not None else float(robot.data.time)
                if viewer is not None and not viewer.is_running():
                    state = 'STOPPED'
                    telemetry = robot.telemetry()
                    publish('RL simulation window closed')
                    return 0
                robot.step(0. if stopping_at is not None else requested)
                if robot.counter % robot.decimation == 0:
                    telemetry = robot.telemetry()
                    if telemetry['fallen']:
                        state = 'FALLEN'
                        publish('RL fall detected; simulation stopped. Start a new walk to reset.')
                        return 2
                if stopping_at is not None:
                    state = 'STOPPING'
                    # The supplied policy may continue stepping at zero command.
                    # Wait for low translational speed, then close the simulation.
                    slow = robot.command_speed <= 1e-6 and robot.planar_speed < .12
                    stable_for = stable_for + robot.model.opt.timestep if slow else 0.
                    if stable_for >= 1.0 and robot.data.time-stopping_at >= 2.:
                        state = 'STOPPED'
                        telemetry = robot.telemetry()
                        publish('RL speed reduced to zero; simulation closed')
                        return 0
                    if robot.data.time-stopping_at > 8.:
                        raise RuntimeError('RL deceleration did not settle within 8 simulation seconds; simulation closed.')
                if args.max_seconds and robot.data.time >= args.max_seconds and stopping_at is None:
                    stopping_at = float(robot.data.time)
                if now >= next_publish:
                    telemetry = robot.telemetry()
                    publish('Decelerating' if stopping_at is not None else 'RL walking')
                    next_publish = now + .1
                if viewer is not None and now >= next_viewer:
                    viewer.sync()
                    next_viewer = now + 1/30
                if not args.fast:
                    next_tick += robot.model.opt.timestep
                    delay = next_tick-time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                    else:
                        next_tick = time.perf_counter()
    except Exception as exc:
        state = 'ERROR'
        traceback.print_exc()
        telemetry['available'] = False
        publish(str(exc))
        return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--session-dir', required=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--fast', action='store_true', help='Headless validation without wall-clock pacing')
    parser.add_argument('--max-seconds', type=float, default=0.)
    sys.exit(run(parser.parse_args()))
