"""Reproduce the small simulation-only RGB can classifier (MUJOCO_GL=egl headless)."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json
import mujoco
import numpy as np
import torch
from backend.can_vision import camera_frame,extract_can,create_network,CLASSES,ROOT,WIDTH,HEIGHT
from scripts.grab_can import CanPickup

def main():
    torch.set_num_threads(2);torch.manual_seed(42)
    rng=np.random.default_rng(42);demo=CanPickup()
    renderer=mujoco.Renderer(demo.model,height=HEIGHT,width=WIDTH)
    crops=[];labels=[]
    try:
        for label,kind in enumerate(CLASSES):
            demo.configure_can(kind if kind!='unknown' else 'cola')
            q=demo.model.joint('can_free').qposadr[0]
            for i in range(100):
                demo.data.qpos[q:q+3]=demo.CAN_CENTER+[rng.uniform(-.025,.025),rng.uniform(-.03,.03),-.001]
                yaw=rng.uniform(-np.pi,np.pi)
                demo.data.qpos[q+3:q+7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
                if kind=='unknown':
                    demo.model.geom_rgba[demo.can_geom]=[rng.uniform(.02,.2),rng.uniform(.45,.95),rng.uniform(.05,.35),1]
                mujoco.mj_forward(demo.model,demo.data)
                candidate=extract_can(*camera_frame(renderer,demo.model,demo.data))
                if candidate is None: raise RuntimeError('Missing training crop')
                crop=candidate['crop'].astype(np.float32)/255
                crop=np.clip(crop*rng.uniform(.8,1.2)+rng.normal(0,.012,crop.shape),0,1)
                crops.append(crop.transpose(2,0,1));labels.append(label)
            print(kind,flush=True)
    finally: renderer.close()
    x=torch.tensor(np.array(crops),dtype=torch.float32);y=torch.tensor(labels)
    train=torch.tensor([i for i in range(len(y)) if i%5]);valid=torch.tensor([i for i in range(len(y)) if not i%5])
    model=create_network();optimizer=torch.optim.Adam(model.parameters(),lr=.002)
    for epoch in range(45):
        model.train()
        order=train[torch.randperm(len(train))]
        for batch in order.split(40):
            optimizer.zero_grad();loss=torch.nn.functional.cross_entropy(model(x[batch]),y[batch]);loss.backward();optimizer.step()
        if epoch%10==0:print(epoch,float(loss.detach()),flush=True)
    model.eval()
    with torch.inference_mode():
        probabilities=model(x[valid]).softmax(1);pred=probabilities.argmax(1)
    metrics={kind:float((pred[y[valid]==i]==i).float().mean()) for i,kind in enumerate(CLASSES)}
    output=ROOT/'vision_assets';output.mkdir(exist_ok=True)
    torch.save(model.state_dict(),output/'can_classifier.pt')
    (output/'training.json').write_text(json.dumps(dict(seed=42,train_samples=len(train),validation_samples=len(valid),accuracy=metrics,scope='Synthetic four-preset MuJoCo RGB crops; unknown examples are green cans. Not validated on real brands or arbitrary objects.'),indent=2))
    print(metrics, 'minimum validation confidence',float(probabilities.max(1).values.min()))
if __name__=='__main__':main()
