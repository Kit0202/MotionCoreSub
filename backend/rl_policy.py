"""CPU inference for the user's supplied Unitree G1 12-joint policy.

Observation layout, recurrent policy and PD gains follow the supplied deployment.
The robot moves only through joint torques and MuJoCo physics.
"""
from pathlib import Path
import math
import numpy as np
import mujoco
import torch
import yaml

ASSETS = Path(__file__).resolve().parents[1] / 'rl_assets'


def roll_pitch(quat):
    w, x, y, z = quat
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(float(np.clip(2*(w*y-z*x), -1, 1))))


def gravity_orientation(quat):
    w, x, y, z = quat
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)])


class G1Policy:
    def __init__(self, assets=ASSETS, model=None):
        assets = Path(assets)
        cfg = yaml.safe_load((assets/'g1_motioncore.yaml').read_text(encoding='utf-8'))
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(assets/cfg['xml_path']))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = float(cfg['simulation_dt'])
        self.decimation = int(cfg['control_decimation'])
        if model is None and (self.model.nu, self.model.nq, self.model.nv) != (12, 19, 18):
            raise ValueError('The supplied RL policy requires its G1 12-joint model.')
        expected = [s+'_'+j+'_joint' for s in ('left', 'right') for j in
                    ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]
        actual = [self.model.joint(int(self.model.actuator_trnid[i, 0])).name for i in range(12)]
        if actual != expected:
            raise ValueError('RL actuator order does not match the trained policy.')
        self.leg_q = np.array([self.model.joint(n).qposadr[0] for n in expected])
        self.leg_v = np.array([self.model.joint(n).dofadr[0] for n in expected])
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(assets/cfg['policy_path']), map_location='cpu')
        self.policy.eval()
        if hasattr(self.policy, 'reset_memory'):
            self.policy.reset_memory()
        self.cfg = cfg
        self.kp, self.kd = (np.asarray(cfg[n], dtype=np.float32) for n in ('kps', 'kds'))
        self.default = np.asarray(cfg['default_angles'], dtype=np.float32)
        self.action = np.zeros(12, dtype=np.float32)
        self.target = self.default.copy()
        self.obs = np.zeros(47, dtype=np.float32)
        self.counter = 0
        self.requested_speed = 0.0
        self.command_speed = 0.0
        self.measured_speed = 0.0
        self.planar_speed = 0.0
        self.floor = self.model.geom('floor').id
        self.feet = [self.model.body(s+'_ankle_roll_link').id for s in ('left', 'right')]
        mujoco.mj_forward(self.model, self.data)

    def step(self, speed, ramp=True, balance_command=None):
        speed = float(speed)
        if not math.isfinite(speed) or not 0 <= speed <= 1:
            raise ValueError('RL forward speed must be finite and between 0 and 1 m/s.')
        self.requested_speed = speed
        dt = self.model.opt.timestep
        # Limit acceleration/deceleration to avoid abrupt command jumps.
        self.command_speed += float(np.clip(speed-self.command_speed, -.5*dt, .5*dt)) if ramp else speed-self.command_speed
        d = self.data
        if self.counter % self.decimation == 0:
            self.obs[:3] = d.qvel[3:6]*self.cfg['ang_vel_scale']
            self.obs[3:6] = gravity_orientation(d.qpos[3:7])
            self.obs[6:9] = np.asarray([self.command_speed, 0, 0] if balance_command is None else balance_command)*self.cfg['cmd_scale']
            self.obs[9:21] = (d.qpos[self.leg_q]-self.default)*self.cfg['dof_pos_scale']
            self.obs[21:33] = d.qvel[self.leg_v]*self.cfg['dof_vel_scale']
            self.obs[33:45] = self.action
            phase = 2*math.pi*((self.counter*dt) % .8)/.8
            self.obs[45:47] = [math.sin(phase), math.cos(phase)]
            with torch.inference_mode():
                self.action = self.policy(torch.from_numpy(self.obs).unsqueeze(0)).cpu().numpy().reshape(12).copy()
            if not np.isfinite(self.action).all():
                raise RuntimeError('Policy produced non-finite joint commands.')
            self.target = self.action*self.cfg['action_scale']+self.default
        d.ctrl[:12] = (self.target-d.qpos[self.leg_q])*self.kp-d.qvel[self.leg_v]*self.kd
        mujoco.mj_step(self.model, d)
        self.counter += 1
        w, x, y, z = d.qpos[3:7]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        forward = math.cos(yaw)*d.qvel[0]+math.sin(yaw)*d.qvel[1]
        alpha = 1-math.exp(-dt/.3)
        self.measured_speed += alpha*(float(forward)-self.measured_speed)
        self.planar_speed += alpha*(float(np.linalg.norm(d.qvel[:2]))-self.planar_speed)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('Simulation state became non-finite.')

    def telemetry(self):
        d, m = self.data, self.model
        roll, pitch = roll_pitch(d.qpos[3:7])
        contact = [False, False]
        force = [0., 0.]
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 == self.floor:
                body = int(m.geom_bodyid[c.geom2])
            elif c.geom2 == self.floor:
                body = int(m.geom_bodyid[c.geom1])
            else:
                continue
            for j, foot in enumerate(self.feet):
                if body == foot:
                    contact[j] = True
                    wrench = np.zeros(6)
                    mujoco.mj_contactForce(m, d, i, wrench)
                    force[j] += abs(float(wrench[0]))
        return dict(height=float(d.qpos[2]), roll=roll, pitch=pitch,
                    leftFoot=contact[0], rightFoot=contact[1], leftForce=force[0], rightForce=force[1],
                    fallen=bool(d.qpos[2] < .5 or max(abs(roll), abs(pitch)) > .8),
                    available=True, source='unitree_rl', x=float(d.qpos[0]), y=float(d.qpos[1]),
                    measuredSpeed=self.measured_speed, planarSpeed=self.planar_speed, requestedSpeed=self.requested_speed,
                    commandSpeed=self.command_speed, simulationTime=float(d.time))
