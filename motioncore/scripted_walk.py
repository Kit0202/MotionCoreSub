"""Slow, flat-ground MuJoCo gait using foot/COM IK and support compensation.

Only actuator targets are written to the live simulation. IK uses separate
MjData; no root translation, external support force, or physics bypass is used.
This is a finite simulation demo, not a hardware or terrain locomotion policy.
"""
from __future__ import annotations

import math
import mujoco
import numpy as np


class ScriptedWalk:
    def __init__(self, robot, speed=0.5):
        self.robot = robot
        self.model, self.data = robot.model, robot.data
        m, d = self.model, self.data
        self.config = robot.config['balanced_walk']
        self.speed = float(np.clip(speed, 0.0, 1.0))
        self.stop_requested = False
        self.phase_name = 'preparing'
        self.k = mujoco.MjData(m)
        self.k.qpos[:] = d.qpos
        self.feet = [m.body(s + '_ankle_roll_link').id for s in ('left', 'right')]
        self.pelvis = m.body('pelvis').id
        names = [s + '_' + j + '_joint' for s in ('left', 'right') for j in
                 ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]
        self.jids = np.array([m.joint(n).id for n in names])
        self.qa = m.jnt_qposadr[self.jids]
        self.va = m.jnt_dofadr[self.jids]
        self.aids = np.array([m.actuator(n).id for n in names])
        self.cols = np.r_[np.arange(3), self.va]
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))
        self.jc = np.zeros((3, m.nv))
        self.mass = float(np.sum(m.body_mass))
        # Retain the current heading and world position on consecutive walks.
        w, x, y, z = d.qpos[3:7]
        self.yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.forward = self.rotation[:2, 0]
        self.lateral = self.rotation[:2, 1]
        self.k.qpos[3:7] = [math.cos(self.yaw/2), 0, 0, math.sin(self.yaw/2)]
        # Bend the knees only in the IK seed to avoid the straight-leg singularity.
        self.k.qpos[self.qa[[0, 3, 4, 6, 9, 10]]] = [-.25, .5, -.25, -.25, .5, -.25]
        self.k.qpos[2] = self.config['pelvis_height']
        mujoco.mj_forward(m, self.k)
        self.targets = d.ctrl[self.aids].copy()
        self.compensation = np.zeros(12)
        self.tick = 0
        self.completed_steps = 0

    @staticmethod
    def smooth(t):
        return t*t*t*(10+t*(-15+6*t))

    def _ik(self, feet, com, height):
        m, k = self.model, self.k
        for _ in range(8):
            mujoco.mj_kinematics(m, k)
            mujoco.mj_comPos(m, k)
            matrices, errors = [], []
            for body, target in zip(self.feet, feet):
                mujoco.mj_jacBody(m, k, self.jp, self.jr, body)
                actual = k.xmat[body].reshape(3, 3)
                # World-frame orientation residual toward a level foot.
                error_r = .5 * sum((np.cross(actual[:, j], self.rotation[:, j])
                                    for j in range(3)), np.zeros(3))
                matrices.extend([self.jp[:, self.cols], self.jr[:, self.cols]])
                errors.extend([target-k.xpos[body], error_r])
            mujoco.mj_jacSubtreeCom(m, k, self.jc, self.pelvis)
            matrices.extend([self.jc[:2, self.cols], np.eye(1, len(self.cols), 2)])
            errors.extend([com-k.subtree_com[self.pelvis, :2], np.array([height-k.qpos[2]])])
            jac, error = np.vstack(matrices), np.concatenate(errors)
            delta = np.linalg.solve(jac.T@jac + np.eye(len(self.cols))*1e-5, jac.T@error)
            k.qpos[:3] += np.clip(delta[:3], -.05, .05)
            k.qpos[self.qa] += np.clip(delta[3:], -.1, .1)
            k.qpos[self.qa] = np.clip(k.qpos[self.qa],
                                      m.jnt_range[self.jids, 0], m.jnt_range[self.jids, 1])
            if np.max(np.abs(error)) < 1e-5:
                break
        return k.qpos[self.qa].copy()

    def _support_compensation(self, feet, swing):
        m, k = self.model, self.k
        mujoco.mj_forward(m, k)
        tau = k.qfrc_bias.copy()
        span = np.dot(feet[0][:2]-feet[1][:2], self.lateral)
        weight = np.clip(np.dot(k.subtree_com[self.pelvis, :2]-feet[1][:2],
                                self.lateral)/max(span, .05), 0, 1)
        weights = [weight, 1-weight]
        if swing is not None:
            weights = [float(i != swing) for i in range(2)]
        for i, body in enumerate(self.feet):
            mujoco.mj_jacBody(m, k, self.jp, self.jr, body)
            force = np.array([0, 0, self.mass*abs(m.opt.gravity[2])*weights[i]])
            moment = np.cross(k.subtree_com[self.pelvis]-k.xpos[body], force)
            tau -= self.jp.T@force + self.jr.T@moment
        # The shipped G1 uses position actuators: convert feedforward torque
        # into a target offset. Physical actuator force limits still apply.
        return tau[self.va]/m.actuator_gainprm[self.aids, 0]

    def _command(self, feet, com, height, swing):
        m, d = self.model, self.data
        if self.tick % 5 == 0:
            correction = np.clip(.6*(com-d.subtree_com[self.pelvis, :2])
                                 -.15*d.qvel[:2], -.04, .04)
            self.targets = self._ik(feet, com+correction, height)
            self.compensation = self._support_compensation(feet, swing)
        commands = self.targets+self.compensation
        w, x, y, z = d.qpos[3:7]
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(np.clip(2*(w*y-z*x), -1, 1))
        commands[[5, 11]] += np.clip(.5*roll+.08*d.qvel[3], -.1, .1)
        commands[[4, 10]] += np.clip(.5*pitch+.08*d.qvel[4], -.1, .1)
        d.ctrl[self.aids] = np.clip(commands, m.actuator_ctrlrange[self.aids, 0],
                                  m.actuator_ctrlrange[self.aids, 1])
        self.tick += 1

    def _phase(self, feet0, feet1, com0, com1, seconds, swing=None, height0=None):
        count = max(1, round(seconds/self.model.opt.timestep))
        height1 = self.config['pelvis_height']
        if height0 is None:
            height0 = height1
        for i in range(count):
            t = (i+1)/count
            s = self.smooth(t)
            feet = [a+(b-a)*s for a, b in zip(feet0, feet1)]
            if swing is not None:
                feet[swing][2] += self.config['foot_clearance']*math.sin(math.pi*t)**2
            com = com0+(com1-com0)*s
            self._command(feet, com, height0+(height1-height0)*s, swing)
            yield

    def run(self):
        d, cfg = self.data, self.config
        feet = [d.xpos[b].copy() for b in self.feet]
        # Flat-floor target for the ankle origin in the included G1 model.
        for foot in feet:
            foot[2] = cfg['ankle_ground_height']
        com = d.subtree_com[self.pelvis, :2].copy()
        yield from self._phase(feet, feet, com, com, 3.0, height0=float(d.qpos[2]))
        for step in range(int(cfg['steps'])):
            if self.stop_requested:
                break
            swing, stance = step % 2, 1-step % 2
            self.phase_name = 'weight transfer'
            support = feet[stance][:2]+cfg['com_forward_offset']*self.forward
            # Pace is deliberately conservative; it is not a promised m/s speed.
            shift_time = cfg['shift_seconds_slow'] + self.speed*(cfg['shift_seconds_fast']-cfg['shift_seconds_slow'])
            swing_time = cfg['swing_seconds_slow'] + self.speed*(cfg['swing_seconds_fast']-cfg['swing_seconds_slow'])
            yield from self._phase(feet, feet, com, support, shift_time)
            com = support
            if self.stop_requested:
                break
            target = [f.copy() for f in feet]
            advance = np.dot(feet[stance][:2]-feet[swing][:2], self.forward)+cfg['step_length']
            target[swing][:2] += advance*self.forward
            self.phase_name = 'left step' if swing == 0 else 'right step'
            print(f'Balanced walk step {step+1}/{cfg["steps"]}: {self.phase_name}')
            yield from self._phase(feet, target, com, com, swing_time, swing=swing)
            feet = target
            self.phase_name = 'landing'
            yield from self._phase(feet, feet, com, com, 1.0)
            self.completed_steps += 1
        self.phase_name = 'settling'
        center = np.mean(feet, axis=0)[:2]+cfg['com_forward_offset']*self.forward
        yield from self._phase(feet, feet, com, center, 4.0)
        yield from self._phase(feet, feet, center, center, 3.0)
        self.phase_name = 'complete'
