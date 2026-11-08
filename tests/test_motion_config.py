import pytest

pytest.importorskip("mujoco")

from motioncore.robot import G1Robot


def test_motion_config_loaded():
    robot = G1Robot()

    assert robot.config is not None
    assert "raise_arm" in robot.config
    assert "wave" in robot.config
    assert "salute" in robot.config


def test_raise_arm_config_sections():
    robot = G1Robot()

    config = robot.config["raise_arm"]

    assert "left" in config
    assert "right" in config
    assert "steps" in config

    for side in ["left", "right"]:
        assert "shoulder_pitch" in config[side]
        assert "shoulder_roll" in config[side]
        assert "elbow" in config[side]


def test_wave_config_sections():
    robot = G1Robot()

    config = robot.config["wave"]

    assert "repetitions" in config
    assert "wrist_yaw_high" in config
    assert "wrist_yaw_low" in config
    assert "steps" in config


def test_salute_config_sections():
    robot = G1Robot()

    config = robot.config["salute"]

    assert "left" in config
    assert "right" in config
    assert "steps" in config
    assert "hold_steps" in config

    for side in ["left", "right"]:
        assert "shoulder_pitch" in config[side]
        assert "shoulder_roll" in config[side]
        assert "elbow" in config[side]
        assert "wrist_yaw" in config[side]


def test_raise_arm_values_within_actuator_limits():
    robot = G1Robot()

    config = robot.config["raise_arm"]

    for side in ["left", "right"]:
        targets = {
            f"{side}_shoulder_pitch_joint": config[side]["shoulder_pitch"],
            f"{side}_shoulder_roll_joint": config[side]["shoulder_roll"],
            f"{side}_elbow_joint": config[side]["elbow"]
        }

        for joint_name, value in targets.items():
            actuator = robot.actuators[joint_name]

            assert actuator["min"] <= value <= actuator["max"]


def test_salute_values_within_actuator_limits():
    robot = G1Robot()

    config = robot.config["salute"]

    for side in ["left", "right"]:
        targets = {
            f"{side}_shoulder_pitch_joint": config[side]["shoulder_pitch"],
            f"{side}_shoulder_roll_joint": config[side]["shoulder_roll"],
            f"{side}_elbow_joint": config[side]["elbow"],
            f"{side}_wrist_yaw_joint": config[side]["wrist_yaw"]
        }

        for joint_name, value in targets.items():
            actuator = robot.actuators[joint_name]

            assert actuator["min"] <= value <= actuator["max"]