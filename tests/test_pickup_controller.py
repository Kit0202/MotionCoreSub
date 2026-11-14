"""API-driven can pickup, physical hold, pause/resume, place and scene reset."""
import numpy as np
import pytest
from fastapi.testclient import TestClient

pytest.importorskip('mujoco')
from backend.pickup_controller import PickupController
from backend import server


def advance_until(controller, status, limit=15000):
    for _ in range(limit):
        controller.step()
        assert controller.status not in {'ERROR','FALLEN'}, controller.get_status()
        if controller.status == status:
            return
    pytest.fail(f'Did not reach {status}: {controller.get_status()}')


def test_mobile_api_pickup_pause_hold_place_reset(monkeypatch):
    controller = PickupController(auto_start=False, viewer=False)
    model, data = controller.demo.model, controller.demo.data
    monkeypatch.setattr(server,'create_controller',lambda:controller)
    with TestClient(server.app) as client:
        def action(name):
            return client.post('/robot/action',json={'action':name}).json()
        status = client.get('/robot/status').json()
        assert status['scene'] == 'pickup' and 'GRAB' in status['availableActions']
        assert client.get('/health').json()['simulationAvailable']
        assert not action('PLACE')['success']
        assert not action('WALK')['success']
        assert not client.post('/robot/speed',json={'speed':.5}).json()['success']
        for _ in range(1327):controller.step()
        assert action('GRAB')['status'] == 'SCANNING'
        assert not action('GRAB')['success']
        assert not action('RESET')['success']
        for _ in range(2137):controller.step()
        assert client.post('/robot/stop').json()['success']
        clock, targets, sim_time = controller._clock, controller.demo.ctrl.copy(), float(data.time)
        for _ in range(333):controller.step()
        assert controller._clock == clock
        np.testing.assert_array_equal(controller.demo.ctrl,targets)
        assert data.time > sim_time+.6
        assert action('RESUME')['status'] == 'GRABBING'
        while controller._clock < 16.:
            controller.step()
        assert client.post('/robot/stop').json()['success']
        for _ in range(317):controller.step()
        assert action('RESUME')['success']
        advance_until(controller,'HOLDING')
        status = client.get('/robot/status').json()
        assert status['holding'] and 'PLACE' in status['availableActions']
        assert 'GRAB' not in status['availableActions']
        for _ in range(1000):controller.step()
        assert controller.status == 'HOLDING'
        telemetry = client.get('/robot/telemetry').json()
        assert telemetry['holding'] and telemetry['thumbForce'] > .05
        assert telemetry['fingerForce'] > .05 and not telemetry['fallen']
        assert telemetry['canHeight'] > controller.demo.CAN_CENTER[2]+.08
        assert client.post('/robot/stop').json()['status'] == 'STOPPED'
        assert 'PLACE' in client.get('/robot/status').json()['availableActions']
        assert action('PLACE')['status'] == 'PLACING'
        for _ in range(500):controller.step()
        assert client.post('/robot/stop').json()['success']
        for _ in range(125):controller.step()
        assert action('RESUME')['status'] == 'PLACING'
        advance_until(controller,'READY')
        assert controller.demo.hand_contacts()['table'] > 3.
        assert 'GRAB' in client.get('/robot/status').json()['availableActions']
        assert action('RESET')['status'] == 'READY'
        assert controller.demo.model is model and controller.demo.data is data
        assert data.time == 0.
        np.testing.assert_allclose(data.body('cola_can').xpos,controller.demo.CAN_CENTER)
        assert 'GRAB' in client.get('/robot/status').json()['availableActions']
        assert not controller.demo.samples


def test_out_of_reach_can_blocks_grasp_before_motion():
    controller=PickupController(auto_start=False,viewer=False)
    try:
        address=controller.demo.model.joint('can_free').qposadr[0]
        controller.demo.data.qpos[address] += .2
        assert controller.execute_action('GRAB')['success']
        for _ in range(6100):controller.step()
        assert controller.status == 'READY'
        assert controller.vision['status'] in {'OUT_OF_REACH','NOT_FOUND','TARGET_NOT_FOUND'}
        assert controller._mode is None and not controller._used
        assert not controller.execute_action('PLACE')['success']
        assert controller.execute_action('RESET')['success']
        assert controller.get_status()['status'] == 'READY'
    finally:
        controller.shutdown()


def test_scene_factory_chooses_pickup_without_initializing_motion(monkeypatch):
    from types import SimpleNamespace
    import sys
    marker=object()
    monkeypatch.setenv('MOTIONCORE_SCENE','pickup')
    monkeypatch.setitem(sys.modules,'backend.pickup_controller',
                        SimpleNamespace(PickupController=lambda:marker))
    assert server.create_controller() is marker
    monkeypatch.setenv('MOTIONCORE_SCENE','wrong')
    with pytest.raises(ValueError, match='MOTIONCORE_SCENE'):
        server.create_controller()
