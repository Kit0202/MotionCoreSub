import mujoco
from pathlib import Path

model_path = Path("models/unitree_g1/scene.xml")
model = mujoco.MjModel.from_xml_path(str(model_path))

print("Number of keyframes:", model.nkey)

for i in range(model.nkey):
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_KEY,
        i
    )
    print(f"{i}: {name}")