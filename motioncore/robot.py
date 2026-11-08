from motioncore.interfaces.mujoco_g1_interface import MuJoCoG1Interface
from motioncore.pose import PoseManager
from motioncore.trajectory import TrajectoryEngine
from motioncore.motion import MotionManager
from motioncore.sequencer import MotionSequencer
from motioncore.state_machine import RobotStateMachine
from motioncore.logger import MotionLogger
from motioncore.config_loader import ConfigLoader
from motioncore.stability import StabilityMonitor
from motioncore.contact import FootContactDetector

class G1Robot:
    """
    High-level MotionCore robot class.

    This class does not directly depend on MuJoCo logic.
    It uses a robot interface, so later we can replace MuJoCo
    with Unitree SDK2 for real robot control.
    """

    def __init__(self, interface=None):

        if interface is None:
            self.interface = MuJoCoG1Interface()
        else:
            self.interface = interface

        self.pose = PoseManager(self)
        self.trajectory = TrajectoryEngine(self)
        self.sequencer = MotionSequencer(self)
        self.logger = MotionLogger()

        self.config = ConfigLoader().load_motion_config()

        self.motion = MotionManager(self)
        self.state_machine = RobotStateMachine(self)
        self.stability = StabilityMonitor(self)
        self.foot_contacts = FootContactDetector(self)

        print("MotionCore G1 Robot Initialized!")

    @property
    def model(self):
        return self.interface.model

    @property
    def data(self):
        return self.interface.data

    @property
    def actuators(self):
        return self.interface.actuators

    def move_joint(self, joint_name, position):
        self.interface.move_joint(joint_name, position)

    def step(self):
        self.interface.step()

    def reset_to_keyframe(self, keyframe_name="home"):
        self.interface.reset_to_keyframe(keyframe_name)
        
    def get_joint_position(self, joint_name):
        return self.interface.get_joint_position(joint_name)
    
    def get_body_height(self, body_name="pelvis"):
        return self.interface.get_body_height(body_name)

    def get_body_quaternion(self, body_name="pelvis"):
        return self.interface.get_body_quaternion(body_name)
    
    def get_body_position(self, body_name="pelvis"):
        return self.interface.get_body_position(body_name)