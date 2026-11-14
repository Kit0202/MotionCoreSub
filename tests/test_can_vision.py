"""Real RGB-D rendering and CNN inference; no mocked recognition results."""
import mujoco
import numpy as np
import pytest
from fastapi.testclient import TestClient
from backend.can_vision import CanVision,camera_frame,WIDTH,HEIGHT
from backend.pickup_controller import PickupController
from backend import server


def test_camera_recognises_pixels_and_estimates_moved_can():
    controller=PickupController(auto_start=False,viewer=False)
    from scripts.grab_can import CanPickup
    demo=CanPickup()
    renderer=mujoco.Renderer(demo.model,height=HEIGHT,width=WIDTH)
    detector=CanVision()
    try:
        for kind in ('cola','tuna','100plus','pepsi'):
            demo.configure_can(kind)
            q=demo.model.joint('can_free').qposadr[0]
            demo.data.qpos[q:q+2] += [.012,-.013]
            mujoco.mj_forward(demo.model,demo.data)
            # Wrong scene metadata must not affect image classification.
            demo.can_type='incorrect_metadata'
            result,jpeg=detector.detect(*camera_frame(renderer,demo.model,demo.data))
            assert result['detectedCan']==kind, result
            assert result['status']=='DETECTED'
            np.testing.assert_allclose(result['position'],demo.data.xpos[demo.can_body],atol=.004)
            assert jpeg[:2]==b'\xff\xd8'
        demo.model.geom_rgba[demo.can_geom]=[.04,.8,.1,1]
        result,_=detector.detect(*camera_frame(renderer,demo.model,demo.data))
        assert result['status']=='UNKNOWN',result
        demo.data.qpos[q]=1.5
        mujoco.mj_forward(demo.model,demo.data)
        result,_=detector.detect(*camera_frame(renderer,demo.model,demo.data))
        assert result['status']=='NOT_FOUND'
    finally:renderer.close();controller.shutdown()


def test_scan_stop_reset_camera_api_and_unknown_blocks_motion(monkeypatch):
    controller=PickupController(auto_start=False,viewer=False)
    monkeypatch.setattr(server,'create_controller',lambda:controller)
    with TestClient(server.app) as client:
        assert client.get('/robot/camera').status_code==404
        def action(a): return client.post('/robot/action',json={'action':a}).json()
        assert action('GRAB')['status']=='SCANNING'
        assert not controller.get_status()['canSelectionAllowed']
        action('STOP');controller.step()
        assert controller._mode is None and controller.vision['status']=='CANCELLED'
        assert not controller._used
        assert action('SCAN')['success'];controller.step()
        assert controller.vision['detectedCan']=='cola'
        assert controller._mode is None and controller.status=='READY'
        response=client.get('/robot/camera')
        assert response.status_code==200 and response.headers['content-type']=='image/jpeg'
        assert response.headers['cache-control']=='no-store'
        frame=controller.vision['frameId']
        assert client.get('/robot/camera').content==response.content
        assert controller.vision['frameId']==frame
        action('RESET')
        assert client.get('/robot/camera').status_code==404
        controller.demo.model.geom_rgba[controller.demo.can_geom]=[.04,.8,.1,1]
        action('GRAB');controller.step()
        assert controller.vision['status']=='TARGET_NOT_FOUND'
        assert controller._mode is None and not controller._used
        controller.demo.model.geom_rgba[controller.demo.can_geom]=[.82,.015,.025,1]
        # A failed rendering/inference must never fall back to scene-selected type.
        monkeypatch.setattr(controller._detector,'detect',lambda *a: (_ for _ in ()).throw(RuntimeError('test failure')))
        action('GRAB');controller.step()
        assert controller.vision['status']=='ERROR' and controller._mode is None
