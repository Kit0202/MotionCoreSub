"""A free can must be supported by opposing contacts, then released onto the table."""
import numpy as np
import pytest

pytest.importorskip('mujoco')
from scripts.grab_can import CanPickup


@pytest.fixture(scope='module')
def completed_pickup():
    demo = CanPickup()
    min_height = float('inf')
    while demo.data.time < 25.:
        demo.step()
        min_height = min(min_height, demo.data.qpos[2])
        assert not demo.failure
        assert np.isfinite(demo.data.qpos).all()
        assert not np.any(demo.data.xfrc_applied)
        assert not np.any(demo.data.qfrc_applied)
    return demo, min_height


def test_can_is_lifted_with_opposing_contacts_and_no_attachment(completed_pickup):
    demo, min_height = completed_pickup
    assert demo.model.neq == 0
    assert demo.model.nmocap == 0
    assert demo.model.joint('can_free').type[0] == 0
    assert min_height > .70
    assert demo.success
    assert len(demo.samples) == 60
    for sample in demo.samples:
        assert sample['height'] > demo.CAN_CENTER[2]+.08
        assert sample['thumb'] > .05 and sample['fingers'] > .05
        assert sample['table'] == 0
        assert sample['opposing']


def test_opening_hand_releases_can_onto_table(completed_pickup):
    demo, _ = completed_pickup
    contacts = demo.hand_contacts()
    assert contacts['thumb'] == contacts['fingers'] == 0
    assert contacts['table'] > 3.
    assert abs(demo.data.xpos[demo.can_body, 2]-(demo.TABLE_TOP+.0575)) < .005
    assert demo.data.body('cola_can').xmat.reshape(3,3)[2,2] > .98


def test_missing_can_contact_cancels_lift():
    demo = CanPickup()
    address = demo.model.joint('can_free').qposadr[0]
    # Different initial placement; the can stays freely simulated thereafter.
    demo.data.qpos[address] += .2
    while demo.data.time < 12.1:
        demo.step()
    assert demo.state == 'FAILED'
    assert not demo.success
    assert 'Lift cancelled' in demo.failure
