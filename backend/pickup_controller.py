"""Phone-controlled pickup in one persistent articulated-hand scene."""
import math
from contextlib import nullcontext
import os
import threading
import time

import mujoco
import numpy as np

from backend.standing import StandingController
from scripts.grab_can import CanPickup
from backend.can_catalog import CAN_TYPES


class PickupController:
    def __init__(self, *, auto_start=True, viewer=None):
        self._lock = threading.RLock()
        self._quit = threading.Event()
        self._thread = None
        self._viewer = None
        self.viewer_enabled = (os.getenv('MOTIONCORE_VIEWER', '0') == '1') if viewer is None else viewer
        self.demo = CanPickup(multi_cans=True)
        self._initial_qpos = self.demo.data.qpos.copy()
        self._initial_ctrl = self.demo.ctrl.copy()
        self._initial_hand = self.demo.initial_hand.copy()
        self.status = 'READY'
        self.message = 'Ready. Pick up the can, then Place can to return it.'
        self.error = None
        self._mode = None
        self._paused = None
        self._clock = 0.
        self._used = False
        self._holding = False
        self._lost_time = 0.
        self._renderer = self._detector = None
        self._frame_id = 0
        self._jpeg = None
        self.vision = dict(status='IDLE', message='Scan the can using the simulated RGB-D camera.', frameId=0)
        self._capture()
        if auto_start:
            self._thread = threading.Thread(target=self._run, name='motioncore-pickup', daemon=True)
            self._thread.start()

    def _view_lock(self):
        viewer = self._viewer
        return viewer.lock() if viewer is not None else nullcontext()

    def _reset(self):
        d, m = self.demo.data, self.demo.model
        mujoco.mj_resetData(m, d)
        d.qpos[:] = self._initial_qpos
        d.ctrl[:] = self._initial_ctrl
        self.demo.ctrl[:] = self._initial_ctrl
        mujoco.mj_forward(m, d)
        self.demo.standing = StandingController(self.demo)
        self.demo.initial_hand = self._initial_hand.copy()
        self.demo.samples.clear()
        self.demo.close_contacts.clear()
        self.demo._hold_ticks = 0
        self.demo.failure = None
        self.demo.success = False
        self.demo.max_height = self.demo.CAN_CENTER[2]
        self.demo.state = 'SETTLE'
        self.demo.set_vision_target(self.demo.can_type,self.demo.CAN_CENTER)
        self.demo.initial_hand = d.xpos[self.demo.hand].copy()
        self._mode = self._paused = None
        self._clock = self._lost_time = 0.
        self._used = self._holding = False
        self._jpeg = None
        self.vision = dict(status='IDLE', message='Scene changed. Scan again.', frameId=0)
        self.status = 'READY'
        self.message = 'Scene reset. Robot and can returned to their initial positions.'
        self._capture()

    def _actions(self):
        if self.error:
            return []
        if self.status in {'ERROR', 'FALLEN'}:
            return ['STOP', 'RESET']
        if self._mode is not None:
            return ['STOP']
        if self._paused is not None:
            return ['STOP', 'RESUME', 'RESET']
        if self._holding:
            return ['STOP', 'PLACE', 'RESET']
        return ['STOP', 'STAND', 'RESET'] + ([] if self._used else ['SCAN', 'GRAB'])

    def _can_select(self):
        return bool(not self.error and self.status in {'READY','STOPPED'}
                    and self._mode is None and self._paused is None and not self._holding)

    def select_can(self, can_type):
        with self._lock:
            if can_type not in CAN_TYPES:
                return {'success':False, 'message':'Unknown can type.'}
            if not self._can_select():
                return {'success':False, 'message':'Place the can or reset the scene before changing can type.'}
            self.demo.select_target(can_type)
            self._used = False
            self._jpeg = None
            self.vision = dict(status='IDLE',message='Target changed. Find the selected can.',frameId=0)
            self.message = f'{self.demo.spec.label} is the pickup target. All four cans remain on the table.'
            self._capture()
            return {'success':True, 'selectedCan':can_type, 'message':self.message}

    def execute_action(self, action, speed=0.):
        action = action.strip().upper()
        with self._lock:
            if action not in self._actions():
                return {'success': False, 'message': 'That action is unavailable now. '+self.message}
            if action == 'RESET':
                with self._view_lock():
                    self._reset()
            elif action == 'STOP':
                self._paused = (None if self._mode in {'SCAN','DETECT_GRAB'} else self._mode) or self._paused
                if self._mode in {'SCAN','DETECT_GRAB'}:
                    self.vision = dict(status='CANCELLED', message='Scan cancelled.', frameId=0)
                    self._jpeg = None
                self._mode = None
                if self.status not in {'ERROR', 'FALLEN'}:
                    self.status = 'STOPPED'
                    self.message = 'Motion paused. The hand keeps its current grip; physics and balance remain active.'
            elif action == 'RESUME':
                self._mode, self._paused = self._paused, None
                self.status = 'GRABBING' if self._mode == 'GRAB' else 'PLACING'
                self.message = 'Continuing the paused can motion.'
            elif action in {'GRAB','SCAN'}:
                self._clock = 0.
                self._mode = 'DETECT_GRAB' if action == 'GRAB' else 'SCAN'
                self.status = 'SCANNING'
                self.vision = dict(status='SCANNING', message='Capturing RGB and depth.', frameId=0)
                self._jpeg = None
                self.message = 'Recognising the can before moving.'
            elif action == 'PLACE':
                self._clock = 18.
                self._mode = 'PLACE'
                self.status = 'PLACING'
                self.message = 'Releasing over the table with clearance from the other cans.'
            elif action == 'STAND':
                self.status = 'READY'
                self.message = 'Standing with both feet supported.'
            return {'success': True, 'status': self.status, 'message': self.message}

    def _scan(self):
        """Called on the simulation thread, which owns the OpenGL context."""
        pickup = self._mode == 'DETECT_GRAB'
        self._mode = None
        try:
            from backend.can_vision import CanVision, camera_frame, WIDTH, HEIGHT
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.demo.model, height=HEIGHT, width=WIDTH)
            if self._detector is None:
                self._detector = CanVision()
            result, jpeg = self._detector.detect(*camera_frame(self._renderer,self.demo.model,self.demo.data),target=self.demo.can_type)
            self._frame_id += 1
            self.vision = {**result, 'frameId':self._frame_id}
            self._jpeg = jpeg
            self.status = 'READY'
            self.message = result['message']
            if pickup and result['status'] == 'DETECTED':
                x,y,z = result['position']
                if not (.33 <= x <= .43 and -.25 <= y <= .25 and .87 <= z <= .93):
                    self.message = 'Detected can is outside the tested arm workspace. Pickup blocked.'
                    self.vision['message'] = self.message
                    self.vision['status'] = 'OUT_OF_REACH'
                    return
                if result['detectedCan'] != self.demo.can_type:
                    self.vision['status'] = 'TARGET_NOT_FOUND'
                    self.message = 'Requested type was not found. Pickup blocked.'
                    return
                self.demo.set_vision_target(result['detectedCan'],result['position'])
                self.demo.initial_hand = self.demo.data.xpos[self.demo.hand].copy()
                self.demo.samples.clear()
                self.demo.close_contacts.clear()
                self.demo._hold_ticks = 0
                self.demo.success = False
                self.demo.failure = None
                self.demo.state = 'SETTLE'
                self.demo.max_height = self.demo.CAN_CENTER[2]
                self._clock = 0.
                self._mode = 'GRAB'
                self._used = True
                self.status = 'GRABBING'
                self.message = 'Reaching for the camera-detected can.'
        except Exception as exc:
            self.status = 'READY'
            self.message = f'Camera scan failed: {exc}. Install requirements-vision.txt and retry.'
            self.vision = dict(status='ERROR',message=self.message,frameId=0)
            self._jpeg = None

    def get_camera_image(self):
        with self._lock:
            return self._jpeg

    def change_speed(self, speed):
        return {'success': False, 'message': 'Walking speed is unavailable in the can-pickup scene.'}

    def _capture(self):
        m, d = self.demo.model, self.demo.data
        w,x,y,z = d.qpos[3:7]
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
        feet = [False, False]
        forces = [0., 0.]
        floor = m.geom('floor').id
        for i in range(d.ncon):
            contact = d.contact[i]
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            body = int(m.geom_bodyid[other])
            for j, foot in enumerate(self.demo.standing.feet):
                if body == foot:
                    feet[j] = True
                    wrench = np.zeros(6)
                    mujoco.mj_contactForce(m,d,i,wrench)
                    forces[j] += abs(float(wrench[0]))
        contact = self.demo.hand_contacts()
        can_height = float(d.xpos[self.demo.can_body,2])
        self._holding = bool(contact['opposing'] and contact['table'] < .1
                             and can_height > self.demo.CAN_CENTER[2]+.07)
        self._telemetry = dict(height=float(d.qpos[2]), roll=roll, pitch=pitch,
                               leftFoot=feet[0], rightFoot=feet[1],
                               leftForce=forces[0], rightForce=forces[1],
                               fallen=bool(d.qpos[2] < .6 or max(abs(roll), abs(pitch)) > .6),
                               available=not bool(self.error), source='pickup_mujoco',
                               timestamp=time.time(), simulationTime=float(d.time),
                               x=float(d.qpos[0]), y=float(d.qpos[1]),
                               canHeight=can_height, holding=self._holding,
                               thumbForce=contact['thumb'], fingerForce=contact['fingers'])

    def step(self):
        with self._lock:
            if self.error:
                return
            dt = self.demo.model.opt.timestep
            if self._mode in {'SCAN','DETECT_GRAB'}:
                self._scan()
            if self._mode is not None:
                self.demo.step(task_time=self._clock)
                self._clock = round(self._clock+dt, 9)
                self.message = self.demo.state.replace('_',' ').title()
                if self._mode == 'GRAB' and self._clock >= 18.:
                    self._mode = None
                    if self.demo.success:
                        self.status = 'HOLDING'
                        self.message = 'Can held. Choose Place can to return it to the table.'
                    else:
                        self.status = 'ERROR'
                        self.message = 'Pickup was not verified. Reset the scene before retrying.'
                elif self._mode == 'PLACE' and self._clock >= 25.:
                    self._mode = None
                    self.status = 'READY'
                    self._used = False
                    self.message = 'Can placed. Choose any target for the next pickup.'
            else:
                self.demo.advance_controls()
            self._capture()
            if self.demo.failure or self._telemetry['fallen']:
                self._mode = self._paused = None
                self.status = 'FALLEN' if self._telemetry['fallen'] else 'ERROR'
                self.message = self.demo.failure or 'Fall detected. Reset the scene.'
            elif self.status == 'HOLDING':
                self._lost_time = 0. if self._holding else self._lost_time+dt
                if self._lost_time > .5:
                    self.status = 'ERROR'
                    self.message = 'The can is no longer held. Reset the scene.'

    def get_status(self):
        with self._lock:
            return dict(status='ERROR' if self.error else self.status,
                        motion=self.demo.state if self._mode else self.status,
                        speed=0., connected=not bool(self.error), controller='CAN_PICKUP',
                        message=self.error or self.message, scene='pickup',
                        availableActions=self._actions(), holding=self._holding,
                        simulationAvailable=not bool(self.error), rlReady=False,
                        phase=self.demo.state,
                        vision=dict(self.vision),
                        tableCans=list(CAN_TYPES),
                        selectedCan=self.demo.can_type,
                        canTypes=[spec.public() for spec in CAN_TYPES.values()],
                        canSelectionAllowed=self._can_select(),
                        progress=(min(1., self._clock/18.) if self._mode == 'GRAB' else
                                  min(1., max(0., (self._clock-18.)/7.)) if self._mode == 'PLACE' else None))

    def get_telemetry(self):
        with self._lock:
            return {**self._telemetry, 'available':not bool(self.error)}

    def _run(self):
        viewer = None
        try:
            if self.viewer_enabled:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.demo.model,self.demo.data)
                self._viewer = viewer
                viewer.cam.lookat[:] = [.25,0,.85]
                viewer.cam.distance = 2.1
                viewer.cam.azimuth = 145
                viewer.cam.elevation = -12
            next_tick = next_sync = time.perf_counter()
            while not self._quit.is_set():
                self.step()
                now = time.perf_counter()
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    if now >= next_sync:
                        with self._lock:
                            viewer.sync()
                        next_sync = now+1/30
                next_tick += self.demo.model.opt.timestep
                delay = next_tick-time.perf_counter()
                if delay > 0:
                    self._quit.wait(delay)
                else:
                    next_tick = time.perf_counter()
        except Exception as exc:
            self.error = f'Pickup simulation stopped: {exc}'
        finally:
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None
            self._viewer = None
            if viewer is not None:
                viewer.close()
            if not self._quit.is_set() and not self.error:
                self.error = 'Simulation window closed. Restart the backend.'

    def shutdown(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=5.)
        elif self._renderer is not None:
            self._renderer.close()
            self._renderer = None
