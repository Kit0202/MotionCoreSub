"""Can recognition from rendered RGB pixels; localisation from calibrated depth.

The detector receives only images and camera calibration, never a selected can
identifier, geometry ID, object pose or segmentation ground truth.
"""
from io import BytesIO
from pathlib import Path
import math
import numpy as np
from PIL import Image, ImageDraw

WIDTH, HEIGHT = 480, 360
ROOT = Path(__file__).resolve().parents[1]
CLASSES = ('cola','tuna','100plus','pepsi','unknown')


def _candidate(rgb, world, mask, table_height):
    h,w = mask.shape
    if int(mask.sum()) < 80:
        return None
    points = world[mask]
    top_z = float(np.quantile(points[:,2],.99))
    top = points[np.abs(points[:,2]-top_z)<.002]
    if len(top)<20:
        return None
    low,high = np.quantile(top[:,:2],[.01,.99],axis=0)
    center = (low+high)/2
    radius = float(np.mean(high-low)/2)
    height = top_z-table_height-.001
    if not (.04<height<.16 and .018<radius<.055):
        return None
    ys,xs = np.where(mask)
    box = [max(0,int(xs.min())-3),max(0,int(ys.min())-3),
           min(w,int(xs.max())+4),min(h,int(ys.max())+4)]
    masked = np.full_like(rgb,115)
    masked[mask]=rgb[mask]
    patch=Image.fromarray(masked).crop(tuple(box))
    patch.thumbnail((58,58),Image.Resampling.BILINEAR)
    crop=Image.new('RGB',(64,64),(115,115,115))
    crop.paste(patch,((64-patch.width)//2,(64-patch.height)//2))
    return dict(crop=np.asarray(crop), box=box,
                position=[float(center[0]),float(center[1]),float(table_height+height/2)],
                radius=radius,height=float(height),pixels=int(mask.sum()))


def extract_cans(rgb, depth, camera_position, camera_rotation, fovy, table_height=.85):
    """Separate objects in a tabletop XY occupancy grid made only from depth."""
    from scipy.ndimage import binary_dilation, label
    h,w = depth.shape
    f = .5*h/math.tan(math.radians(fovy)*.5)
    v,u = np.indices((h,w))
    optical = np.stack([(u+.5-w/2)*depth/f, -(v+.5-h/2)*depth/f, -depth],axis=-1)
    world = optical @ np.asarray(camera_rotation).reshape(3,3).T + camera_position
    mask = (np.isfinite(depth) & (depth>.08) & (depth<2.) &
            (world[:,:,0]>.30) & (world[:,:,0]<.86) &
            (world[:,:,1]>-.34) & (world[:,:,1]<.34) &
            (world[:,:,2]>table_height+.003) & (world[:,:,2]<table_height+.18))
    if int(mask.sum())<80:
        return []
    indices = np.floor((world[mask,:2]-[.30,-.34])/.004).astype(int)
    grid = np.zeros((142,172),dtype=bool)
    grid[indices[:,0],indices[:,1]]=True
    components,count = label(binary_dilation(grid),structure=np.ones((3,3)))
    pixel_ids = np.zeros((h,w),dtype=int)
    pixel_ids[mask]=components[indices[:,0],indices[:,1]]
    candidates=[]
    for index in range(1,count+1):
        candidate=_candidate(rgb,world,pixel_ids==index,table_height)
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def extract_can(*args, **kwargs):
    """Single-object adapter for reproducible training and legacy evaluation."""
    candidates=extract_cans(*args,**kwargs)
    return candidates[0] if len(candidates)==1 else None


def camera_frame(renderer, model, data):
    import mujoco
    mujoco.mj_forward(model,data)
    renderer.disable_depth_rendering()
    renderer.update_scene(data,camera='can_camera')
    rgb=renderer.render().copy()
    renderer.enable_depth_rendering()
    renderer.update_scene(data,camera='can_camera')
    depth=renderer.render().copy()
    renderer.disable_depth_rendering()
    camera=model.camera('can_camera').id
    return rgb,depth,data.cam_xpos[camera].copy(),data.cam_xmat[camera].reshape(3,3).copy(),float(model.cam_fovy[camera])


def create_network():
    import torch.nn as nn
    return nn.Sequential(
        nn.Conv2d(3,12,5,padding=2),nn.ReLU(),nn.MaxPool2d(2),
        nn.Conv2d(12,24,3,padding=1),nn.ReLU(),nn.MaxPool2d(2),
        nn.Conv2d(24,32,3,padding=1),nn.ReLU(),nn.AdaptiveAvgPool2d((2,2)),
        nn.Flatten(),nn.Linear(128,48),nn.ReLU(),nn.Linear(48,len(CLASSES)),
    )


class CanVision:
    def __init__(self, weights=None):
        import torch
        self.torch=torch
        torch.set_num_threads(1)
        self.model=create_network()
        path=Path(weights) if weights else ROOT/'vision_assets/can_classifier.pt'
        state=torch.load(path,map_location='cpu',weights_only=True)
        self.model.load_state_dict(state)
        self.model.eval()

    def detect(self,rgb,depth,position,rotation,fovy,target=None):
        candidates=extract_cans(rgb,depth,position,rotation,fovy)
        detections=[]
        for candidate in candidates:
            tensor=self.torch.from_numpy(candidate['crop'].copy()).permute(2,0,1).float().unsqueeze(0)/255.
            with self.torch.inference_mode():
                probabilities=self.model(tensor).softmax(1)[0].numpy()
            index=int(np.argmax(probabilities));score=float(probabilities[index])
            known=CLASSES[index]!='unknown' and score>=.90
            detections.append(dict(status='DETECTED' if known else 'UNKNOWN',
                detectedCan=CLASSES[index] if known else None,confidence=score,
                **{k:candidate[k] for k in ('position','radius','height','box')}))
        matches=[d for d in detections if d['detectedCan']==target] if target else detections
        result=dict(status='TARGET_NOT_FOUND' if target else 'NOT_FOUND',
                    detectedCan=None,confidence=0.,position=None,targetCan=target,
                    message='Requested can not confidently found. No arm movement.' if target else 'No can found.')
        if len(matches)==1:
            result.update(matches[0])
            result['message']='Requested can located among the table objects.' if result['status']=='DETECTED' else 'Can type uncertain; pickup blocked.'
        elif len(matches)>1:
            result.update(status='AMBIGUOUS',message='Multiple matching cans found; pickup blocked.')
        result['detections']=detections
        image=Image.fromarray(rgb);draw=ImageDraw.Draw(image)
        for detection in detections:
            selected=detection is (matches[0] if len(matches)==1 else None)
            color='lime' if selected and detection['status']=='DETECTED' else 'cyan' if detection['status']=='DETECTED' else 'orange'
            box=detection['box'];draw.rectangle(tuple(box),outline=color,width=2)
            draw.text((box[0],max(0,box[1]-13)),
                f"{detection['detectedCan'] or 'unknown'} {detection['confidence']:.0%}",fill=color)
        encoded=BytesIO();image.save(encoded,format='JPEG',quality=85)
        return result,encoded.getvalue()
