from motioncore.interfaces.base_interface import RobotInterface


class UnitreeSDK2Interface(RobotInterface):
    """
    Placeholder interface for future real Unitree G1 integration.

    This will later use Unitree SDK2 Python to communicate with
    the physical robot.
    """

    def __init__(self):
        self._actuators = {}

    @property
    def actuators(self):
        return self._actuators

    def move_joint(self, joint_name, position):
        raise NotImplementedError(
            "Unitree SDK2 real robot control is not implemented yet."
        )

    def step(self):
        raise NotImplementedError(
            "Unitree SDK2 update loop is not implemented yet."
        )

    def reset_to_keyframe(self, keyframe_name="stand"):
        raise NotImplementedError(
            "Real robot keyframe reset is not implemented yet."
        )
        
    def get_joint_position(self, joint_name):
        raise NotImplementedError(
            "Unitree SDK2 joint state reading is not implemented yet."
        )
        
    def get_body_height(self, body_name="pelvis"):
        raise NotImplementedError(
            "Unitree SDK2 body height reading is not implemented yet."
        )

    def get_body_quaternion(self, body_name="pelvis"):
        raise NotImplementedError(
            "Unitree SDK2 body orientation reading is not implemented yet."
        )
        
    def get_body_position(self, body_name="pelvis"):
        raise NotImplementedError(
            "Unitree SDK2 body position reading is not implemented yet."
        )