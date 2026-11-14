"""Four simultaneous free-body cans; target selection must never teleport them."""
import numpy as np
import pytest
from fastapi.testclient import TestClient
import mujoco
from backend.can_catalog import CAN_TYPES
from backend.pickup_controller import PickupController
from backend import server


def positions(demo):
    return {kind:demo.data.body(('' if kind=='cola' else kind+'_')+'cola_can').xpos.copy() for kind in CAN_TYPES}


def until(controller,status,limit=13000):
    for _ in range(limit):
        controller.step()
        assert controller.status not in {'ERROR','FALLEN'},f'{controller.demo.can_type}: {controller.message}; close={sum(controller.demo.close_contacts)}/{len(controller.demo.close_contacts)}'
        if controller.status==status:return
    pytest.fail(str(controller.get_status()))


def test_pick_and_place_all_targets_sequentially_without_reset(monkeypatch):
    controller=PickupController(auto_start=False,viewer=False)
    demo=controller.demo
    monkeypatch.setattr(server,'create_controller',lambda:controller)
    with TestClient(server.app) as client:
        assert client.get('/robot/status').json()['tableCans']==list(CAN_TYPES)
        assert demo.model.nq==7+43+4*7 # G1 free base, articulated joints, four free cans
        for kind in CAN_TYPES:
            previous=demo.data.qpos.copy();sim_time=demo.data.time
            result=client.post('/robot/can',json={'canType':kind}).json()
            assert result['success'] and result['selectedCan']==kind
            np.testing.assert_array_equal(previous,demo.data.qpos)
            assert demo.data.time==sim_time
            # Let all objects settle before recording the neighbours.
            for _ in range(100):controller.step()
            before=positions(demo)
            assert client.post('/robot/action',json={'action':'GRAB'}).json()['success']
            assert not controller.select_can('cola')['success']
            until(controller,'HOLDING')
            assert controller.vision['detectedCan']==kind
            assert controller.get_status()['holding']
            assert len(controller.demo.samples)==60
            assert all(s['opposing'] and s['table']==0 for s in demo.samples)
            assert demo.arm_side==('left' if controller.vision['position'][1]>0 else 'right')
            for other,pos in positions(demo).items():
                if other!=kind:np.testing.assert_allclose(pos,before[other],atol=.015,err_msg=f'{kind} disturbed {other}')
            assert client.post('/robot/action',json={'action':'PLACE'}).json()['success']
            until(controller,'READY')
            assert demo.hand_contacts()['table']>CAN_TYPES[kind].mass*8
            assert not controller.get_status()['holding']
            assert controller.get_status()['canSelectionAllowed']
            assert 'GRAB' in controller.get_status()['availableActions']
            for other,pos in positions(demo).items():
                if other!=kind:np.testing.assert_allclose(pos,before[other],atol=.015)
            assert demo.model.neq==demo.model.nmocap==0


def test_selection_validation_pause_and_reset(monkeypatch):
    controller=PickupController(auto_start=False,viewer=False)
    monkeypatch.setattr(server,'create_controller',lambda:controller)
    with TestClient(server.app) as client:
        assert client.post('/robot/can',json={'canType':'unknown'}).status_code==422
        assert not controller.select_can('unknown')['success']
        controller.execute_action('GRAB')
        for _ in range(50):controller.step()
        controller.execute_action('STOP')
        assert not controller.get_status()['canSelectionAllowed']
        assert not controller.select_can('tuna')['success']
        controller.execute_action('RESET')
        assert controller.select_can('100plus')['success']
        controller.execute_action('RESET')
        assert controller.demo.arm_side=='left'
        assert controller.demo.can_type=='100plus'
        np.testing.assert_allclose(controller.demo.initial_hand,controller.demo.data.xpos[controller.demo.hand])
        for kind,pos in positions(controller.demo).items():
            np.testing.assert_allclose(pos[:2],controller.demo.CAN_POSITIONS[kind])


def test_normal_motion_scene_rejects_can_selection(monkeypatch):
    from test_server import FakeController
    monkeypatch.setattr(server,'create_controller',FakeController)
    with TestClient(server.app) as client:
        assert not client.post('/robot/can',json={'canType':'tuna'}).json()['success']
