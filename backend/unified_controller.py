"""One articulated MuJoCo robot, one owner thread, one persistent viewer.

The recurrent leg policy stays active at zero velocity during upper-body gestures.
No model reload, root pinning, state copy, or subprocess occurs on motion changes.
"""
from pathlib import Path
import math
import os
import threading
import time
import numpy as np
import mujoco
from backend.rl_policy import G1Policy


class UnifiedController:
    def __init__(self, *, auto_start=True, viewer=None):
        self._lock = threading.RLock()
        self._quit = threading.Event()
        self._thread = None
        self.viewer_enabled = (os.getenv('MOTIONCORE_VIEWER', '0') == '1') if viewer is None else viewer
        model = mujoco.MjModel.from_xml_path(str(Path(__file__).resolve().parents[1]/'models/unitree_g1/scene_unified.xml'))
        self.robot = G1Policy(model=model)
        self.status = 'IDLE'
        self.motion = 'STAND'
        self.speed = 0.
        self.message = 'One robot ready for walking and gestures'
        self.error = None
        self.pending = None
        self.gesture_time = 0.
        self.still_time = 0.
        self.stop_time = 0.
        self.anchor = self.robot.data.qpos[:2].copy()
        self.heading = 0.
        self.arm_target = np.zeros(model.nu-12)
        self.gesture_start = self.arm_target.copy()
        self.return_start = None
        self._telemetry = self.robot.telemetry()
        if auto_start:
            self._thread = threading.Thread(target=self._run, name='motioncore-unified', daemon=True)
            self._thread.start()

    @staticmethod
    def _speed(value):
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError('Speed must be between 0 and 1 m/s')
        return value

    def _begin(self, action):
        if self.status == 'STOPPING':
            self.anchor = self.robot.data.qpos[:2].copy()
            self.heading = self._yaw()
        self.pending = None
        self.motion = action
        self.gesture_time = 0.
        self.gesture_start = self.arm_target.copy()
        self.return_start = None
        self.status = 'MOVING' if action in {'WAVE', 'SALUTE'} or np.max(np.abs(self.arm_target)) > .05 else ('STOPPED' if action == 'STOP' else 'IDLE')
        self.message = f'{action.title()} in the shared simulation'

    def execute_action(self, action, speed=0.):
        action = action.upper().strip()
        if action not in {'WALK','STOP','STAND','WAVE','SALUTE'}:
            return {'success':False, 'message':'Unknown action'}
        try:
            speed = self._speed(speed)
        except (ValueError, TypeError) as exc:
            return {'success':False, 'message':str(exc)}
        with self._lock:
            if self.error:
                return {'success':False, 'message':self.error}
            if self.status == 'FALLEN':
                if action != 'STAND':
                    return {'success':False, 'message':'Fall detected. Use Stand to reset upright.'}
                # Recovery only: retain horizontal position. Normal transitions never reset.
                xy = self.robot.data.qpos[:2].copy()
                mujoco.mj_resetData(self.robot.model, self.robot.data)
                self.robot.data.qpos[:2] = xy
                self.robot.counter = 0
                self.robot.action[:] = 0
                self.robot.command_speed = self.robot.measured_speed = self.robot.planar_speed = 0.
                if hasattr(self.robot.policy, 'reset_memory'):
                    self.robot.policy.reset_memory()
                self.arm_target[:] = 0
                self.speed = 0.
                self.anchor = xy.copy()
                self.heading = 0.
                mujoco.mj_forward(self.robot.model, self.robot.data)
                self._begin('STAND')
            elif action == 'WALK' and speed > 0:
                if self.status in {'MOVING','STOPPING'} or np.max(np.abs(self.arm_target)) > .05:
                    return {'success':False, 'message':'Wait for the arms to return to rest before walking.'}
                self.speed = speed
                self.status, self.motion = 'WALKING', 'RL_WALK'
                self.message = 'Walking in the same window'
            elif self.status in {'WALKING','STOPPING'}:
                self.speed = 0.
                if self.status != 'STOPPING':
                    self.stop_time = self.still_time = 0.
                self.status = 'STOPPING'
                self.pending = 'STOP' if action == 'WALK' else action
                self.message = f'Slowing down before {self.pending.lower()}'
            else:
                self.speed = 0.
                self._begin('STOP' if action == 'WALK' else action)
            return {'success':True, 'message':self.message, 'status':self.status, 'controller':'UNITREE_RL'}

    def change_speed(self, speed):
        try:
            speed = self._speed(speed)
        except (ValueError, TypeError) as exc:
            return {'success':False, 'message':str(exc)}
        with self._lock:
            if speed == 0:
                return {**self.execute_action('STOP'), 'speed':0., 'applied':True}
            if self.status != 'WALKING':
                return {'success':False, 'message':'Start walking before changing live speed'}
            self.speed = speed
            return {'success':True, 'speed':speed, 'applied':True, 'message':'Walking speed updated'}

    def _arms(self, dt):
        self.gesture_time += dt
        t = self.gesture_time
        goal = np.zeros_like(self.arm_target)
        if self.motion in {'WAVE','SALUTE'}:
            side = 'left' if self.motion == 'WAVE' else 'right'
            values = ({'shoulder_pitch':-1.5, 'shoulder_roll':.5, 'elbow':0., 'wrist_yaw':0.}
                      if self.motion == 'WAVE' else
                      {'shoulder_pitch':-1.3, 'shoulder_roll':-.3, 'elbow':-.7, 'wrist_yaw':.3})
            if self.motion == 'WAVE':
                values['wrist_yaw'] = .45*math.sin(2*math.pi*(t-1.5)) if 1.5<t<4.5 else 0.
            for joint, value in values.items():
                goal[self.robot.model.actuator(side+'_'+joint+'_joint').id-12] = value
            if t < 1.5:
                u = t/1.5; u=u*u*(3-2*u)
                goal = self.gesture_start*(1-u)+goal*u
            elif t >= 4.5:
                if self.return_start is None:
                    self.return_start = self.arm_target.copy()
                u = min(1., (t-4.5)/1.5); u=u*u*(3-2*u)
                goal = self.return_start*(1-u)
            if t >= 6.:
                self._begin('STAND')
                goal[:] = 0
        else:
            # Smooth cancellation/return; no abrupt arm target jump on Stop.
            u = min(1., t/1.5); u=u*u*(3-2*u)
            goal = self.gesture_start*(1-u)
            if t >= 1.5 and self.status == 'MOVING':
                self.status = 'STOPPED' if self.motion == 'STOP' else 'IDLE'
        self.arm_target[:] = goal
        self.robot.data.ctrl[12:] = goal

    def _yaw(self):
        w,x,y,z = self.robot.data.qpos[3:7]
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def _balance_command(self):
        # Position hold through learned leg torques, never by moving/pinning the root.
        yaw = self._yaw()
        c,s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c,s],[-s,c]])
        velocity = rotation @ (.8*(self.anchor-self.robot.data.qpos[:2])-.2*self.robot.data.qvel[:2])
        turn = math.atan2(math.sin(self.heading-yaw), math.cos(self.heading-yaw))
        return [*np.clip(velocity, -.3, .3), float(np.clip(turn, -.3, .3))]

    def step(self):
        """Deterministic single physics step; also used by headless regression tests."""
        with self._lock:
            if self.status == 'FALLEN' or self.error:
                return
            dt = self.robot.model.opt.timestep
            self._arms(dt)
            self.robot.step(self.speed, balance_command=None if self.status in {'WALKING','STOPPING'} else self._balance_command())
            self._telemetry = {**self.robot.telemetry(), 'timestamp':time.time(), 'source':'unified_mujoco'}
            if self._telemetry['fallen']:
                self.status, self.motion, self.speed = 'FALLEN','STOP',0.
                self.message = 'Fall detected. Use Stand to reset upright.'
                self.pending = None
            elif self.status == 'STOPPING':
                self.stop_time += dt
                stable = self.robot.command_speed < .001 and self.robot.planar_speed < .12
                self.still_time = self.still_time+dt if stable else 0.
                if self.still_time >= 1. and self.stop_time >= 2.:
                    self._begin(self.pending or 'STOP')
                elif self.stop_time > 12.:
                    # Keep balancing, never begin a gesture while still moving.
                    self.message = 'Still settling; gesture will begin once speed is low'

    def get_status(self):
        with self._lock:
            return dict(status='ERROR' if self.error else self.status, motion=self.motion,
                        speed=self.speed, controller='UNITREE_RL', connected=not bool(self.error),
                        walkMode='rl', message=self.error or self.message,
                        simulationState=self.status, simulationAvailable=not bool(self.error),
                        rlReady=not bool(self.error), rlRunning=self.status in {'WALKING','STOPPING'},
                        speedApplied=abs(self.robot.requested_speed-self.speed)<1e-6, unified=True)

    def get_telemetry(self):
        with self._lock:
            return {**self._telemetry, 'available':not bool(self.error)}

    def _run(self):
        viewer = None
        try:
            if self.viewer_enabled:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.robot.model, self.robot.data)
                viewer.cam.distance = 2.7
            next_tick = time.perf_counter()
            next_sync = next_tick
            while not self._quit.is_set():
                self.step()
                now = time.perf_counter()
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    if now >= next_sync:
                        with self._lock:
                            viewer.cam.lookat[:] = self.robot.data.qpos[:3]
                            viewer.sync()
                        next_sync = now+1/30
                next_tick += self.robot.model.opt.timestep
                delay = next_tick-time.perf_counter()
                if delay > 0:
                    self._quit.wait(delay)
                else:
                    next_tick = time.perf_counter()
        except Exception as exc:
            with self._lock:
                self.error = f'Unified simulation stopped: {exc}'
        finally:
            if viewer is not None:
                viewer.close()
            if not self._quit.is_set() and not self.error:
                self.error = 'Simulation window closed. Restart the backend to reopen it.'

    def shutdown(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=5.)
