from abc import ABC, abstractmethod


class RobotInterface(ABC):
    """
    Abstract robot interface for MotionCore.

    Any robot backend, such as MuJoCo simulation or Unitree SDK2,
    should implement this interface.
    """

    @property
    @abstractmethod
    def actuators(self):
        pass

    @abstractmethod
    def move_joint(self, joint_name, position):
        pass

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def reset_to_keyframe(self, keyframe_name="stand"):
        pass
    
    @abstractmethod
    def get_joint_position(self, joint_name):
        pass
    
    @abstractmethod
    def get_body_height(self, body_name="pelvis"):
        pass

    @abstractmethod
    def get_body_quaternion(self, body_name="pelvis"):
        pass
    
    @abstractmethod
    def get_body_position(self, body_name="pelvis"):
        pass