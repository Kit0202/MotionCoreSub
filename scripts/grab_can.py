"""Standing tabletop pickup with the supplied articulated G1 hand.

Run with --headless for repeatable physics validation, or without it for a viewer.
The can is a free body. No weld, attachment, root pinning or support force is used.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse
import copy
from collections import deque
import json
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from backend.standing import StandingController
from backend.can_catalog import CAN_TYPES, FONT


class CanPickup:
    TABLE_TOP = .85
    CAN_CENTER = np.array([.40, -.18, TABLE_TOP+.059])
    GRIP_OFFSET = np.array([.13, .04, 0.])

    # Spaced tabletop pairs on either side of the robot.
    CAN_POSITIONS = {'cola':(.40,-.23), 'tuna':(.35,-.10),
                     '100plus':(.35,.10), 'pepsi':(.40,.23)}

    def __init__(self, can_type='cola', *, multi_cans=False):
        self.multi_cans = multi_cans
        if can_type not in CAN_TYPES:
            raise ValueError('Unknown can type: '+str(can_type))
        self.spec = CAN_TYPES[can_type]
        self.can_type = can_type
        self.CAN_CENTER = np.array([.40, -.18, self.TABLE_TOP+self.spec.half_height+.0015])
        self.GRIP_OFFSET = np.array(self.spec.grip_offset)
        root = ET.parse(ROOT/'models/unitree_g1/g1_with_hands.xml').getroot()
        root.find('compiler').set('meshdir', str(ROOT/'models/unitree_g1/assets'))
        root.find('option').set('timestep', '.002')
        root.find('option').set('iterations', '100')
        key = root.find('keyframe')
        if key is not None:
            root.remove(key)
        for index, actuator in enumerate(root.find('actuator')):
            if index < 12:
                actuator.tag = 'motor'
                actuator.attrib.clear()
                names = [s+'_'+j+'_joint' for s in ('left','right') for j in
                         ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')]
                actuator.set('name', names[index]); actuator.set('joint', names[index])
            else:
                actuator.set('kp', '8' if 'hand_' in actuator.get('name', '') else '80')
                actuator.set('dampratio', '1')
        for joint in root.iter('joint'):
            if 'hand_' in joint.get('name', ''):
                joint.set('armature', '.0001'); joint.set('frictionloss', '.01')
                joint.set('damping', '.02')
        for body in root.iter('body'):
            if 'hand_' in body.get('name', ''):
                for geom in body.findall('geom'):
                    if geom.get('class') == 'collision':
                        geom.set('friction', '1.2 .02 .001'); geom.set('condim', '4')
        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'headlight', ambient='.25 .25 .25', diffuse='.7 .7 .7')
        ET.SubElement(root.find('asset'), 'texture', type='skybox', builtin='gradient',
                      rgb1='.3 .5 .7', rgb2='.05 .08 .12', width='512', height='3072')
        # Head-mounted RGB-D camera: render RGB and metric depth from one pose.
        direction = np.array([.27, -.03 if multi_cans else -.18, -.37])
        direction /= np.linalg.norm(direction)
        right = np.cross(direction, [0., 0., 1.])
        right /= np.linalg.norm(right)
        up = np.cross(right, direction)
        torso = next(body for body in root.iter('body') if body.get('name') == 'torso_link')
        ET.SubElement(torso, 'camera', name='can_camera', pos='.13 0 .43',
                      xyaxes=' '.join(map(str, np.r_[right,up])), fovy='75' if multi_cans else '60')
        world = root.find('worldbody')
        ET.SubElement(world, 'geom', name='floor', type='plane', size='0 0 .1', rgba='.18 .23 .28 1')
        ET.SubElement(world, 'geom', name='table', type='box', pos=f'.58 0 {self.TABLE_TOP-.025}', size='.28 .34 .025',
                      rgba='.45 .28 .12 1', friction='1 .01 .001')
        for x in [.34,.82]:
            for y in [-.29,.29]:
                ET.SubElement(world, 'geom', type='box', pos=f'{x} {y} {(self.TABLE_TOP-.05)/2}', size=f'.025 .025 {(self.TABLE_TOP-.05)/2}', rgba='.25 .25 .25 1')
        can = ET.SubElement(world, 'body', name='cola_can', pos=' '.join(map(str,self.CAN_CENTER)))
        ET.SubElement(can, 'freejoint', name='can_free')
        ET.SubElement(can, 'geom', name='can', type='cylinder', size=f'{self.spec.radius} {self.spec.half_height}', mass=str(self.spec.mass),
                      rgba=' '.join(map(str,self.spec.rgba)), friction='1.2 .02 .001', condim='4', solref='.008 1')
        for index, z in enumerate([-self.spec.half_height,self.spec.half_height]):
            ET.SubElement(can, 'geom', name=f'can_rim_{index}', type='cylinder', size=f'{self.spec.radius} .001', pos=f'0 0 {z}',
                          rgba='.8 .8 .82 1', contype='0', conaffinity='0', mass='.001')
        for index in range(7*3*5):
            ET.SubElement(can, 'geom', name=f'can_label_{index}', type='box',
                          size='.0004 .001 .001', rgba='1 1 1 0',
                          contype='0', conaffinity='0', density='0')
        if multi_cans:
            for kind in CAN_TYPES:
                if kind == 'cola':
                    continue
                other = copy.deepcopy(can)
                for element in other.iter():
                    if 'name' in element.attrib:
                        element.set('name',kind+'_'+element.get('name'))
                world.append(other)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        m,d = self.model,self.data
        self.ctrl = np.zeros(m.nu)
        for side, sign in [('left',1),('right',-1)]:
            for joint, angle in [('shoulder_pitch',.1),('shoulder_roll',.12*sign),('elbow',1.05)]:
                aid = m.actuator(side+'_'+joint+'_joint').id
                self.ctrl[aid] = angle
                d.qpos[m.jnt_qposadr[m.actuator_trnid[aid,0]]] = angle
        d.ctrl[:] = self.ctrl
        mujoco.mj_forward(m,d)
        self.standing = StandingController(SimpleNamespace(model=m,data=d))
        self.k = mujoco.MjData(m)
        self.arm_names = ['right_'+j+'_joint' for j in
                         ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]
        self.arm_a = np.array([m.actuator(n).id for n in self.arm_names])
        self.arm_j = m.actuator_trnid[self.arm_a,0]
        self.arm_q,self.arm_v = m.jnt_qposadr[self.arm_j],m.jnt_dofadr[self.arm_j]
        self.arm_side = 'right'
        self.hand = m.body('right_wrist_yaw_link').id
        self.can_body,self.can_geom = m.body('cola_can').id,m.geom('can').id
        self.jp,self.jr = np.zeros((3,m.nv)),np.zeros((3,m.nv))
        self.raise_hand = np.array([.10, -.30, 1.02])
        self.pre = self.CAN_CENTER-self.GRIP_OFFSET+np.array([0,-.10,.03])
        self.reach = self.CAN_CENTER-self.GRIP_OFFSET
        self.lift = self.reach+np.array([0,0,.13])
        self.initial_hand = d.xpos[self.hand].copy()
        self.max_height = self.CAN_CENTER[2]
        self.state = 'SETTLE'
        self.success = False
        self.samples = []
        self._hold_ticks = 0
        self.close_contacts = deque(maxlen=100)
        self.failure = None
        if multi_cans:
            for kind in CAN_TYPES:
                self.configure_can(kind)
            self.select_target(can_type)
        else:
            self.configure_can(can_type)

    def can_name(self, name):
        return self.can_type+'_'+name if self.multi_cans and self.can_type != 'cola' else name

    def select_target(self, can_type):
        """Bind contact verification to the requested object without moving any object."""
        if can_type not in CAN_TYPES:
            raise ValueError('Unknown can type')
        self.can_type = can_type
        self.spec = CAN_TYPES[can_type]
        self.can_body = self.model.body(self.can_name('cola_can')).id
        self.can_geom = self.model.geom(self.can_name('can')).id
        xy = self.CAN_POSITIONS[can_type] if self.multi_cans else (.40,-.18)
        self.CAN_CENTER = np.array([*xy,self.TABLE_TOP+self.spec.half_height+.0015])
        self.set_vision_target(can_type,self.CAN_CENTER)
        self.max_height = self.CAN_CENTER[2]

    def configure_can(self, can_type):
        """Reconfigure at a scene reset; retain the model and viewer data objects."""
        if can_type not in CAN_TYPES:
            raise ValueError('Unknown can type: '+str(can_type))
        spec = CAN_TYPES[can_type]
        m,d = self.model,self.data
        self.select_target(can_type)
        self.grasp_spec = spec
        self.GRIP_OFFSET = np.asarray(spec.grip_offset)
        self.pre = self.CAN_CENTER-self.GRIP_OFFSET+np.array([0,-.10,.03])
        self.reach = self.CAN_CENTER-self.GRIP_OFFSET
        self.lift = self.reach+np.array([0,0,.13])
        m.geom_size[self.can_geom,:2] = [spec.radius,spec.half_height]
        for index,z in enumerate([-spec.half_height,spec.half_height]):
            geom = m.geom(self.can_name(f'can_rim_{index}')).id
            m.geom_size[geom,:2] = [spec.radius,.001]
            m.geom_pos[geom,:] = [0,0,z]
        radius,h = spec.radius,spec.half_height*2
        lateral = spec.mass*(3*radius*radius+h*h)/12+2*.001*(radius*radius/4+.002**2/12+spec.half_height**2)
        axial = (spec.mass+.002)*radius*radius/2
        m.body_mass[self.can_body] = spec.mass+.002
        m.body_inertia[self.can_body] = [lateral,lateral,axial]
        m.body_iquat[self.can_body] = [1,0,0,0]
        m.geom_rgba[self.can_geom] = spec.rgba
        can_q = m.joint(self.can_name('can_free')).qposadr[0]
        m.qpos0[can_q:can_q+3] = self.CAN_CENTER
        for index in range(7*3*5):
            m.geom_rgba[m.geom(self.can_name(f'can_label_{index}')).id,3] = 0
        pixel = min(.0028, spec.radius*1.25/(len(spec.text)*4))
        label_color = [.02,.15,.45,1.] if can_type in {'100plus','tuna'} else [1.,1.,1.,1.]
        for letter,char in enumerate(spec.text):
            for row,line in enumerate(FONT[char].split('/')):
                for col,value in enumerate(line):
                    geom = m.geom(self.can_name(f'can_label_{letter*15+row*3+col}')).id
                    y = -((len(spec.text)*4-2)/2-letter*4-col)*pixel
                    m.geom_pos[geom] = [(radius*radius-y*y)**.5+.0004,y,(2-row)*pixel]
                    m.geom_size[geom] = [.0004,pixel*.43,pixel*.43]
                    m.geom_rgba[geom] = label_color if value=='1' else [0,0,0,0]
        mujoco.mj_setConst(m,self.k)
        d.qpos[can_q:can_q+3] = self.CAN_CENTER
        mujoco.mj_forward(m,d)
        self.standing = StandingController(self)
        self.max_height = self.CAN_CENTER[2]

    def set_vision_target(self, can_type, position):
        """Plan the wrist and fingers from camera estimates, independently of scene selection."""
        self.grasp_spec = CAN_TYPES[can_type]
        center = np.asarray(position, dtype=float)
        side = 'left' if self.multi_cans and center[1] > 0 else 'right'
        changed = side != self.arm_side
        self.arm_side = side
        m = self.model
        self.arm_names = [side+'_'+j+'_joint' for j in
            ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]
        self.arm_a = np.array([m.actuator(n).id for n in self.arm_names])
        self.arm_j = m.actuator_trnid[self.arm_a,0]
        self.arm_q,self.arm_v = m.jnt_qposadr[self.arm_j],m.jnt_dofadr[self.arm_j]
        self.hand = m.body(side+'_wrist_yaw_link').id
        sign = -1 if side == 'left' else 1
        angle = -.2*sign if self.multi_cans and can_type=='tuna' else 0.
        c,s = np.cos(angle),np.sin(angle)
        self.hand_rotation = np.array([[1,0,0],[0,c,-s],[0,s,c]])
        offset = np.asarray(self.grasp_spec.grip_offset)*[1,sign,1]
        self.reach = center-offset
        self.raise_hand = np.array([.10,-.30*sign,1.02])
        self.pre = self.reach+np.array([0,0,.15] if self.multi_cans else [0,-.10*sign,.03])
        self.lift = self.reach+np.array([0,0,.13])
        self.release_position = self.lift+np.array([.010 if can_type=='tuna' else .025,-.01*sign,0])
        if changed:
            self.initial_hand = self.data.xpos[self.hand].copy()

    def ik(self, target):
        m,k,d = self.model,self.k,self.data
        k.qpos[:] = d.qpos
        for _ in range(12):
            mujoco.mj_kinematics(m,k);mujoco.mj_comPos(m,k)
            mujoco.mj_jacBody(m,k,self.jp,self.jr,self.hand)
            R = k.xmat[self.hand].reshape(3,3)
            orientation = .5*sum((np.cross(R[:,i],self.hand_rotation[:,i]) for i in range(3)),np.zeros(3))
            error = np.r_[target-k.xpos[self.hand], .3*orientation]
            J = np.vstack([self.jp[:,self.arm_v],.3*self.jr[:,self.arm_v]])
            delta = J.T@np.linalg.solve(J@J.T+np.eye(6)*1e-5,error)
            k.qpos[self.arm_q] = np.clip(k.qpos[self.arm_q]+np.clip(delta,-.12,.12),
                                      m.jnt_range[self.arm_j,0]+.001,m.jnt_range[self.arm_j,1]-.001)
            if np.linalg.norm(error) < 1e-5:break
        return k.qpos[self.arm_q].copy()

    def hand_contacts(self):
        groups = {'thumb':0.,'fingers':0.,'table':0.}
        normals = {'thumb':np.zeros(3), 'fingers':np.zeros(3)}
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            if self.can_geom not in (contact.geom1,contact.geom2):continue
            other = contact.geom2 if contact.geom1==self.can_geom else contact.geom1
            name = self.model.body(int(self.model.geom_bodyid[other])).name
            force = np.zeros(6);mujoco.mj_contactForce(self.model,self.data,i,force)
            group = ('thumb' if name and name.startswith(self.arm_side+'_hand_thumb') else
                     'fingers' if name and name.startswith((self.arm_side+'_hand_index',self.arm_side+'_hand_middle')) else None)
            if group:
                groups[group] += abs(float(force[0]))
                sign = 1 if contact.geom2 == self.can_geom else -1
                normals[group] += sign*contact.frame[:3]*abs(force[0])
            elif other == self.model.geom('table').id:groups['table']+=abs(force[0])
        lengths = np.linalg.norm(normals['thumb'])*np.linalg.norm(normals['fingers'])
        groups['opposing'] = bool(lengths > 1e-6 and np.dot(normals['thumb'],normals['fingers'])/lengths < -.3)
        return groups

    def step(self, task_time=None):
        t = round(self.data.time if task_time is None else task_time, 9)
        stages = [('SETTLE',2.,self.initial_hand,self.initial_hand,0.,0.),
                  ('RAISE',3.,self.initial_hand,self.raise_hand,0.,0.),
                  ('PREGRASP',3.,self.raise_hand,self.pre,0.,0.),
                  ('REACH',2.,self.pre,self.reach,0.,0.),
                  ('CLOSE',2.,self.reach,self.reach,0.,1.),
                  ('LIFT',3.,self.reach,self.lift,1.,1.),
                  ('HOLD',3.,self.lift,self.lift,1.,1.),
                  ('LOWER',3.,self.lift,self.reach,1.,1.),
                  ('RELEASE',2.,self.reach,self.reach,1.,0.),
                  ('RETRACT',2.,self.reach,self.pre,0.,0.)]
        elapsed = t
        for state,duration,a,b,g0,g1 in stages:
            if elapsed < duration:break
            elapsed -= duration
        else:
            state,duration,a,b,g0,g1 = ('DONE',1.,self.pre,self.pre,0.,0.)
            elapsed = 1.
        if state == 'CLOSE':
            contacts = self.hand_contacts()
            self.close_contacts.append(bool(contacts['thumb']>.05 and contacts['fingers']>.05 and contacts['opposing']))
        if state == 'LIFT' and self.state == 'CLOSE':
            # Evaluate a contact window, avoiding a single solver-tick normal flip.
            if len(self.close_contacts)<50 or sum(self.close_contacts)<.6*len(self.close_contacts):
                self.failure = 'No sustained opposing finger contact. Lift cancelled.'
        if self.failure:
            self.state = 'FAILED'
            self.data.ctrl[:12] = self.standing.torque()
            mujoco.mj_step(self.model,self.data)
            return
        self.state = 'RELEASE_CLEARANCE' if self.multi_cans and state=='LOWER' else state
        u = np.clip(elapsed/duration,0,1);u=u*u*(3-2*u)
        target = a+(b-a)*u;grip=g0+(g1-g0)*u
        thumb_grip = finger_grip = grip
        if self.multi_cans and state in {'LOWER','RELEASE'}:
            # Unfold above neighbouring cans. The free body settles onto the
            # table under gravity; this is a clearance release, not insertion.
            target = self.lift+(self.release_position-self.lift)*u if state=='LOWER' else self.release_position
        elif self.multi_cans and state == 'RETRACT':
            target = self.release_position+(self.raise_hand-self.release_position)*u
        if state != 'SETTLE' and self.standing.tick%5==0:
            self.ctrl[self.arm_a] = self.ik(target)
        for joint,closed in zip(('thumb_0','thumb_1','thumb_2','index_0','index_1','middle_0','middle_1'), self.grasp_spec.grip):
            self.ctrl[self.model.actuator(self.arm_side+'_hand_'+joint+'_joint').id] = (thumb_grip if joint.startswith('thumb') else finger_grip)*closed*(-1 if self.arm_side=='left' and joint!='thumb_0' else 1)
        self.advance_controls()
        height = float(self.data.xpos[self.can_body,2])
        if state == 'HOLD':
            self._hold_ticks += 1
        if state=='HOLD' and self._hold_ticks%25==0 and len(self.samples)<60:
            forces=self.hand_contacts()
            self.samples.append(dict(height=height,**forces))
        self.success=bool(len(self.samples)>=60 and all(s['height']>self.CAN_CENTER[2]+.08 and s['thumb']>.05 and s['fingers']>.05 and s['table']<.1 and s['opposing'] for s in self.samples))

    def advance_controls(self):
        self.data.ctrl[12:] = self.ctrl[12:]
        upper = self.model.jnt_dofadr[self.model.actuator_trnid[12:,0]]
        self.data.ctrl[12:] += self.data.qfrc_bias[upper]/self.model.actuator_gainprm[12:,0]
        self.data.ctrl[:12] = self.standing.torque()
        mujoco.mj_step(self.model,self.data)
        height = float(self.data.xpos[self.can_body,2]);self.max_height=max(self.max_height,height)
        w,x,y,z = self.data.qpos[3:7]
        tilt = max(abs(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y))),
                   abs(np.arcsin(np.clip(2*(w*y-z*x),-1,1))))
        if self.data.qpos[2] < .6 or tilt > .6:
            self.failure = 'Robot lost balance. Arm sequence cancelled.'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--can',choices=list(CAN_TYPES),default='cola')
    args=parser.parse_args();demo=CanPickup(args.can)
    def run(viewer=None):
        last=None
        while viewer is not None or demo.data.time<25.:
            start=time.perf_counter();demo.step()
            if demo.state!=last:
                print(demo.state, 'can',np.round(demo.data.xpos[demo.can_body],3),'contacts',demo.hand_contacts(),flush=True);last=demo.state
            if viewer is not None:
                if not viewer.is_running():break
                if demo.standing.tick%15==0:viewer.sync()
                delay=demo.model.opt.timestep-(time.perf_counter()-start)
                if delay>0:time.sleep(delay)
        print(json.dumps(dict(success=demo.success, failure=demo.failure, max_height=demo.max_height,hold_samples=len(demo.samples), hold_min_height=min((s['height'] for s in demo.samples),default=0), contacts=demo.hand_contacts()),indent=2))
    if args.headless:
        run()
        if not demo.success or demo.failure:
            raise SystemExit(1)
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(demo.model,demo.data) as viewer:
            viewer.cam.lookat[:]=[.25,0,.8];viewer.cam.distance=2.2;viewer.cam.azimuth=145;viewer.cam.elevation=-12
            run(viewer)


if __name__=='__main__':main()
