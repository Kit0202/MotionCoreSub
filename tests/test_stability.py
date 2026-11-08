import pytest

pytest.importorskip("mujoco")

from motioncore.robot import G1Robot


def test_stability_monitor_returns_status():
    robot = G1Robot()

    robot.reset_to_keyframe("stand")
    robot.step()

    status = robot.stability.get_stability_status()

    assert "height" in status
    assert "roll" in status
    assert "pitch" in status
    assert "fallen" in status
    assert isinstance(status["fallen"], bool)


def test_robot_not_fallen_in_stand_pose():
    robot = G1Robot()

    robot.reset_to_keyframe("stand")
    robot.step()

    assert robot.stability.is_fallen() is False