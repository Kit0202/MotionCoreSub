"""Physics regressions: forward travel, foot clearance, repeated walk, safe stop."""
import math
import numpy as np
import pytest

pytest.importorskip('mujoco')
from motioncore.robot import G1Robot
from backend.simulation_runtime import MotionCoreSimulation
from backend.robot_controller import RobotController
from test_backend_controller import FakeRL, FakeTelemetry


def advance(robot, sim=None):
    robot.state_machine.update()
    robot.step()
    if sim is not None:
        sim._capture_telemetry_locked()
    assert not robot.stability.is_fallen()


@pytest.mark.parametrize('speed', [0.05, 0.5, 1.0])
def test_forward_steps_and_consecutive_walks(speed):
    robot = G1Robot()
    robot.reset_to_keyframe('stand')
    robot.motion.walk_speed = speed
    for run in range(2):
        start = robot.data.qpos[:3].copy()
        clearance = [0., 0.]
        robot.state_machine.set_state('SLOW_FORWARD_WALK_V1')
        for _ in range(40000):
            advance(robot)
            assert np.isfinite(robot.data.qpos).all()
            assert robot.data.qpos[2] > .70
            assert max(abs(v) for v in robot.stability.get_roll_pitch()) < .12
            for i, side in enumerate(('left', 'right')):
                clearance[i] = max(clearance[i], robot.get_body_height(side+'_ankle_roll_link')-.035)
            if robot.state_machine.is_idle():
                break
        else:
            pytest.fail('walking did not finish')
        for _ in range(1000):
            advance(robot)
        assert robot.data.qpos[0]-start[0] > .25
        assert abs(robot.data.qpos[1]-start[1]) < .05
        assert min(clearance) > .02


@pytest.mark.parametrize('stop_time', [1., 5., 8.5])
def test_stop_during_prepare_transfer_and_swing(stop_time):
    robot = G1Robot()
    sim = MotionCoreSimulation(robot=robot, auto_start=False)
    controller = RobotController(simulation=sim, rl=FakeRL(), telemetry_monitor=FakeTelemetry())
    assert controller.execute_action('WALK', 1.)['success']
    for _ in range(round(stop_time/robot.model.opt.timestep)):
        advance(robot, sim)
    result = controller.execute_action('STOP')
    assert result['success']
    assert controller.get_status()['status'] == 'STOPPING'
    assert not controller.execute_action('WALK', 1.)['success']
    for _ in range(8000):
        advance(robot, sim)
        if robot.state_machine.is_idle():
            break
    assert robot.state_machine.is_idle()
    for _ in range(1000):
        advance(robot, sim)
    assert controller.get_status()['status'] == 'STOPPED'
    contacts = robot.foot_contacts.get_contact_status()
    assert contacts['left_contact'] and contacts['right_contact']
    assert robot.motion.active_walk is None


def test_zero_pace_and_live_pace_update():
    robot = G1Robot()
    sim = MotionCoreSimulation(robot=robot, auto_start=False)
    controller = RobotController(simulation=sim, rl=FakeRL(), telemetry_monitor=FakeTelemetry())
    assert not controller.execute_action('WALK', 0.)['success']
    assert robot.state_machine.is_idle()
    assert controller.execute_action('WALK', .5)['success']
    advance(robot, sim)
    assert controller.change_speed(.8)['applied']
    assert robot.motion.active_walk.speed == .8
    assert controller.change_speed(0.)['success']
    assert controller.get_status()['status'] == 'STOPPING'
