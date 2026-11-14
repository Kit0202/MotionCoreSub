"""Flat-floor double-support joint control; no root pinning or gait inference.

The shared IK solver uses separate MjData. The live model receives only actuator
torques, with gravity distributed between the two planted feet.
"""
from types import SimpleNamespace
import math
import numpy as np
import mujoco
from motioncore.scripted_walk import ScriptedWalk


class StandingController(ScriptedWalk):
    def __init__(self, robot):
        proxy = SimpleNamespace(model=robot.model, data=robot.data,
                                config={'balanced_walk': {'pelvis_height': .76}})
        super().__init__(proxy)
        self.feet_targets = [self.data.xpos[b].copy() for b in self.feet]
        for target in self.feet_targets:
            target[2] = .0331362476
        self.com_target = np.mean(self.feet_targets, axis=0)[:2] + .02*self.forward
        self.targets = self.data.qpos[self.qa].copy()
        self.kp = np.tile([180., 180., 120., 220., 100., 100.], 2)
        self.kd = np.tile([8., 8., 6., 10., 5., 5.], 2)
        self.upper_q = np.array([self.model.jnt_qposadr[int(j)]
                                for j in self.model.actuator_trnid[12:, 0]])
        self.initial_torque = self.data.ctrl[:12].copy()
        self.age = 0.

    def torque(self):
        d, m = self.data, self.model
        if self.tick % 5 == 0:
            self.k.qpos[self.upper_q] = d.qpos[self.upper_q]
            correction = np.clip(.6*(self.com_target-d.subtree_com[self.pelvis, :2])
                                 -.15*d.qvel[:2], -.04, .04)
            self.targets = self._ik(self.feet_targets, self.com_target+correction, .76)
            self.compensation = self._gravity_torque()
        targets = self.targets.copy()
        w,x,y,z = d.qpos[3:7]
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
        targets[[5,11]] += np.clip(.5*roll+.08*d.qvel[3], -.1, .1)
        targets[[4,10]] += np.clip(.5*pitch+.08*d.qvel[4], -.1, .1)
        torque = (targets-d.qpos[self.qa])*self.kp-d.qvel[self.va]*self.kd+self.compensation
        self.age += m.opt.timestep
        self.tick += 1
        blend = self.smooth(min(1., self.age/.05))
        return self.initial_torque*(1-blend)+torque*blend

    def _gravity_torque(self):
        m, k = self.model, self.k
        mujoco.mj_forward(m, k)
        span = np.dot(self.feet_targets[0][:2]-self.feet_targets[1][:2], self.lateral)
        left = float(np.clip(np.dot(k.subtree_com[self.pelvis, :2]-self.feet_targets[1][:2],
                                   self.lateral)/max(span, .05), .1, .9))
        weights = [left, 1-left]
        center = sum(w*p for w,p in zip(weights, self.feet_targets))
        gravity = np.array([0., 0., self.mass*abs(m.opt.gravity[2])])
        moment = np.cross(k.subtree_com[self.pelvis]-center, gravity)
        torque = k.qfrc_bias.copy()
        for body, weight in zip(self.feet, weights):
            mujoco.mj_jacBody(m, k, self.jp, self.jr, body)
            torque -= weight*(self.jp.T@gravity+self.jr.T@moment)
        return torque[self.va]
