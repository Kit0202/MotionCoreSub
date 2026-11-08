import math
import time
import json
import pytest
from backend.rl_adapter import RLLocomotion
from backend.rl_ipc import atomic_json
from backend.robot_controller import RobotController
from test_backend_controller import FakeSimulation, FakeRL


def wait_for(check, timeout=25):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = check()
        if value:
            return value
        time.sleep(.1)
    raise AssertionError('Timed out waiting for RL state')


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -.1, 1.1])
def test_invalid_speed_never_reaches_controller(value):
    controller = RobotController(simulation=FakeSimulation(), rl=FakeRL())
    assert not controller.execute_action('WALK', value)['success']
    assert not controller.change_speed(value)['success']


def test_wrong_session_and_stale_telemetry_are_not_live(tmp_path):
    adapter = RLLocomotion(headless=True, runtime_dir=tmp_path)
    adapter.session_dir = tmp_path
    adapter.token = 'current'
    atomic_json(tmp_path/'telemetry.json', {'session': 'previous', 'timestamp': time.time(), 'available': True})
    assert not adapter.get_telemetry()['available']
    atomic_json(tmp_path/'telemetry.json', {'session': 'current', 'timestamp': time.time()-10, 'available': True})
    assert not adapter.get_telemetry()['available']


def test_failed_process_is_error_not_walking(tmp_path):
    class DeadProcess:
        returncode = 1
        def poll(self): return 1
    adapter = RLLocomotion(headless=True, runtime_dir=tmp_path)
    adapter.session_dir = tmp_path
    adapter.process = DeadProcess()
    assert adapter.status()['status'] == 'ERROR'


@pytest.mark.parametrize('speed', [.2, .5, 1.])
def test_policy_walk_speed_and_deceleration(speed):
    pytest.importorskip('torch')
    pytest.importorskip('mujoco')
    from backend.rl_policy import G1Policy
    robot = G1Policy()
    measured = []
    for i in range(10000):
        robot.step(speed if i < 7500 else 0.)
        if i % 50 == 0:
            sample = robot.telemetry()
            assert not sample['fallen']
            assert sample['height'] > .70
            if 2500 <= i < 7000:
                measured.append(sample['measuredSpeed'])
    assert abs(sum(measured)/len(measured)-speed) < .15
    assert robot.data.qpos[0] > speed*10
    assert robot.telemetry()['commandSpeed'] == 0.
    assert robot.planar_speed < .12


def test_euler_telemetry_uses_angles():
    pytest.importorskip('torch')
    from backend.rl_policy import roll_pitch
    roll, pitch = roll_pitch([math.cos(.2), math.sin(.2), 0, 0])
    assert roll == pytest.approx(.4)
    assert pitch == pytest.approx(0.)


def test_real_worker_start_live_speed_stop_and_gesture(tmp_path):
    pytest.importorskip('torch')
    pytest.importorskip('mujoco')
    adapter = RLLocomotion(headless=True, runtime_dir=tmp_path)
    controller = RobotController(simulation=FakeSimulation(), rl=adapter, walk_mode='rl')
    try:
        result = controller.execute_action('WALK', .3)
        assert result['success'] and result['status'] == 'STARTING'
        assert not controller.execute_action('WALK', .3)['success']
        assert not controller.execute_action('WAVE')['success']
        wait_for(lambda: controller.get_status()['status'] == 'WALKING')
        update = controller.change_speed(.7)
        assert update['success']
        wait_for(lambda: controller.get_telemetry().get('commandSeq', 0) == update['commandSeq'])
        wait_for(lambda: controller.get_telemetry().get('commandSpeed', 0) >= .69)
        assert controller.get_telemetry()['source'] == 'unitree_rl'
        stop = controller.execute_action('STOP')
        assert stop['success']
        assert controller.get_status()['status'] == 'STOPPING'
        assert not controller.execute_action('WAVE')['success']
        wait_for(lambda: not adapter.is_running(), timeout=15)
        assert controller.get_status()['status'] == 'STOPPED'
        assert controller.get_telemetry()['commandSpeed'] == 0.
        assert controller.execute_action('WAVE')['success']
        assert controller.get_telemetry()['source'] == 'mujoco'
        # A new session must not reuse the previous STOP or previous telemetry.
        controller.execute_action('STOP')
        previous = adapter.token
        assert controller.execute_action('WALK', .2)['success']
        assert adapter.token != previous
        controller.execute_action('STOP')
        wait_for(lambda: not adapter.is_running(), timeout=20)
        assert controller.get_status()['status'] == 'STOPPED'
    finally:
        controller.shutdown()
