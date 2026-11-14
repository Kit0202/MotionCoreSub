"""Camera finds a requested type among four objects, independent of its slot."""
import mujoco
import numpy as np
from backend.pickup_controller import PickupController
from backend.can_catalog import CAN_TYPES


def joint(demo,kind):return demo.model.joint(('' if kind=='cola' else kind+'_')+'can_free').qposadr[0]


def scan(c,target,action='SCAN'):
    assert c.select_can(target)['success']
    assert c.execute_action(action)['success']
    c.step()
    return c.vision


def test_all_targets_detected_from_one_table_without_scene_changes():
    c=PickupController(auto_start=False,viewer=False)
    try:
        for kind in CAN_TYPES:
            before=c.demo.data.qpos.copy()
            result=scan(c,kind)
            assert result['status']=='DETECTED',result
            assert result['detectedCan']==kind
            assert {d['detectedCan'] for d in result['detections']}==set(CAN_TYPES)
            np.testing.assert_allclose(result['position'],c.demo.data.xpos[c.demo.can_body],atol=.004)
            assert c.demo.data.time<.02
            assert c._mode is None
    finally:c.shutdown()


def test_camera_picks_requested_type_after_slots_are_swapped():
    c=PickupController(auto_start=False,viewer=False);d=c.demo
    try:
        cola,pepsi=joint(d,'cola'),joint(d,'pepsi')
        d.data.qpos[cola:cola+2],d.data.qpos[pepsi:pepsi+2]=d.data.qpos[pepsi:pepsi+2].copy(),d.data.qpos[cola:cola+2].copy()
        mujoco.mj_forward(d.model,d.data)
        result=scan(c,'cola','GRAB')
        assert result['detectedCan']=='cola' and result['position'][1]>.15
        assert d.arm_side=='left'
        for _ in range(9005):
            c.step()
            assert c.status not in {'ERROR','FALLEN'},c.get_status()
        assert c.status=='HOLDING' and c.get_status()['holding']
        assert d.hand_contacts()['opposing']
    finally:c.shutdown()


def test_missing_and_ambiguous_requested_type_never_fall_back():
    c=PickupController(auto_start=False,viewer=False);d=c.demo
    try:
        cola=joint(d,'cola');d.data.qpos[cola]=1.3
        mujoco.mj_forward(d.model,d.data)
        result=scan(c,'cola','GRAB')
        assert result['status']=='TARGET_NOT_FOUND'
        assert len(result['detections'])==3
        assert c._mode is None and not c._used
        c.execute_action('RESET')
        # Labels/selected IDs still say Pepsi, but the pixels now show another red can.
        d.model.geom('pepsi_can').rgba[:]=CAN_TYPES['cola'].rgba
        result=scan(c,'cola','GRAB')
        assert result['status']=='AMBIGUOUS',result
        assert c._mode is None and not c._used
    finally:c.shutdown()
