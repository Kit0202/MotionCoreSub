import numpy as np
import pytest

from motioncore.trajectory import TrajectoryEngine


class FakeData:
    def __init__(self):
        self.ctrl = np.array([0.0, 0.25], dtype=float)


class FakeRobot:
    def __init__(self):
        self.data = FakeData()
        self.actuators = {
            "joint_a": {"id": 0, "min": -2.0, "max": 2.0},
            "joint_b": {"id": 1, "min": -2.0, "max": 2.0},
        }

    def move_joint(self, name, value):
        self.data.ctrl[self.actuators[name]["id"]] = value


def test_linear_reaches_target():
    robot = FakeRobot()
    samples = list(TrajectoryEngine(robot).linear("joint_a", 1.0, steps=5))
    assert len(samples) == 5
    assert robot.data.ctrl[0] == pytest.approx(1.0)


def test_multi_joint_reaches_targets():
    robot = FakeRobot()
    list(TrajectoryEngine(robot).multi_joint_linear({"joint_a": 1.0, "joint_b": -0.5}, steps=4))
    assert robot.data.ctrl[0] == pytest.approx(1.0)
    assert robot.data.ctrl[1] == pytest.approx(-0.5)


def test_single_step_does_not_divide_by_zero():
    robot = FakeRobot()
    list(TrajectoryEngine(robot).multi_joint_linear({"joint_a": 0.8}, steps=1))
    assert robot.data.ctrl[0] == pytest.approx(0.8)


@pytest.mark.parametrize("steps", [0, -1, 1.5])
def test_invalid_steps_are_rejected(steps):
    robot = FakeRobot()
    with pytest.raises(ValueError):
        list(TrajectoryEngine(robot).linear("joint_a", 0.5, steps=steps))
