import pytest

pytest.importorskip("mujoco")

from motioncore.interfaces.base_interface import RobotInterface
from motioncore.interfaces.mujoco_g1_interface import MuJoCoG1Interface
from motioncore.interfaces.unitree_sdk2_interface import UnitreeSDK2Interface


def test_mujoco_interface_implements_robot_interface():
    interface = MuJoCoG1Interface()

    assert isinstance(interface, RobotInterface)
    assert len(interface.actuators) > 0


def test_mujoco_interface_has_required_methods():
    interface = MuJoCoG1Interface()

    assert hasattr(interface, "move_joint")
    assert hasattr(interface, "step")
    assert hasattr(interface, "reset_to_keyframe")
    assert hasattr(interface, "actuators")


def test_unitree_sdk2_interface_placeholder():
    interface = UnitreeSDK2Interface()

    assert isinstance(interface, RobotInterface)

    with pytest.raises(NotImplementedError):
        interface.move_joint("right_shoulder_pitch_joint", 0.5)
        
def test_mujoco_interface_body_position():
    interface = MuJoCoG1Interface()

    position = interface.get_body_position("pelvis")

    assert len(position) == 3