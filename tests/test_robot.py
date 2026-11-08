import pytest

pytest.importorskip("mujoco")

from motioncore.robot import G1Robot


def test_robot_initializes():
    robot = G1Robot()

    assert robot.model is not None
    assert robot.data is not None
    assert len(robot.actuators) > 0


def test_valid_joint_command():
    robot = G1Robot()

    joint_name = "left_shoulder_pitch_joint"
    target_position = 0.5

    robot.move_joint(joint_name, target_position)

    actuator_id = robot.actuators[joint_name]["id"]

    assert robot.data.ctrl[actuator_id] == target_position


def test_invalid_joint_name():
    robot = G1Robot()

    with pytest.raises(ValueError):
        robot.move_joint("invalid_joint", 0.5)


def test_joint_limit_check():
    robot = G1Robot()

    with pytest.raises(ValueError):
        robot.move_joint("left_shoulder_pitch_joint", 100.0)
        
def test_get_joint_position():
    robot = G1Robot()

    position = robot.get_joint_position("left_shoulder_pitch_joint")

    assert isinstance(position, float)