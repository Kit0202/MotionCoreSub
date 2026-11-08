import pytest

pytest.importorskip("mujoco")

from motioncore.robot import G1Robot


def test_state_machine_initial_state():
    robot = G1Robot()

    assert robot.state_machine.current_state == "IDLE"
    assert robot.state_machine.is_idle()


def test_available_states():
    robot = G1Robot()

    states = robot.state_machine.get_available_states()

    assert "STAND" in states
    assert "WAVE_LEFT_HAND" in states
    assert "WAVE_RIGHT_HAND" in states
    assert "DEMO_SALUTE" in states


def test_invalid_state():
    robot = G1Robot()

    with pytest.raises(ValueError):
        robot.state_machine.set_state("INVALID_STATE")