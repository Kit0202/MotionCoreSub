import mujoco
from pathlib import Path
from motioncore.interfaces.base_interface import RobotInterface

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class MuJoCoG1Interface(RobotInterface):
    """
    MuJoCo interface for the Unitree G1 simulation model.
    This class handles low-level simulator communication.
    """

    def __init__(self, model_path="models/unitree_g1/scene.xml"):
        model_path = Path(model_path).expanduser()
        self.model_path = model_path if model_path.is_absolute() else PROJECT_ROOT / model_path

        if not self.model_path.exists():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        self._actuators = {}

        for i in range(self.model.nu):
            name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                i
            )

            self._actuators[name] = {
                "id": i,
                "min": self.model.actuator_ctrlrange[i][0],
                "max": self.model.actuator_ctrlrange[i][1]
            }

        print("MuJoCo G1 Interface Initialized!")

    def move_joint(self, joint_name, position):
        """Send position command to one actuator."""

        if joint_name not in self._actuators:
            raise ValueError(f"Unknown actuator: {joint_name}")

        actuator = self._actuators[joint_name]

        if position < actuator["min"] or position > actuator["max"]:
            raise ValueError(
                f"{joint_name} position {position} is outside "
                f"range [{actuator['min']}, {actuator['max']}]"
            )

        self.data.ctrl[actuator["id"]] = position

    def step(self):
        """Step the MuJoCo simulation."""
        mujoco.mj_step(self.model, self.data)

    def reset_to_keyframe(self, keyframe_name="home"):
        """Reset robot using MuJoCo keyframe."""

        key_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_KEY,
            keyframe_name
        )

        if key_id == -1:
            raise ValueError(f"Keyframe '{keyframe_name}' not found.")

        self.data.qpos[:] = self.model.key_qpos[key_id]
        self.data.qvel[:] = 0

        if self.model.nu > 0:
            self.data.ctrl[:] = self.model.key_ctrl[key_id]

        mujoco.mj_forward(self.model, self.data)

        print(f"Robot reset to keyframe: {keyframe_name}")
        
    @property
    def actuators(self):
        return self._actuators
    
    def get_joint_position(self, joint_name):
        """Return current joint position from MuJoCo qpos."""

        joint_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name
        )

        if joint_id == -1:
            raise ValueError(f"Unknown joint: {joint_name}")

        qpos_address = self.model.jnt_qposadr[joint_id]

        return float(self.data.qpos[qpos_address])
    
    def get_body_height(self, body_name="pelvis"):
        """Return body height from MuJoCo world position."""

        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name
        )

        if body_id == -1:
            raise ValueError(f"Unknown body: {body_name}")

        return float(self.data.xpos[body_id][2])


    def get_body_quaternion(self, body_name="pelvis"):
        """Return body orientation quaternion from MuJoCo."""

        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name
        )

        if body_id == -1:
            raise ValueError(f"Unknown body: {body_name}")

        return self.data.xquat[body_id].copy()
    
    def get_body_position(self, body_name="pelvis"):
        """Return body world position from MuJoCo."""

        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_name
        )

        if body_id == -1:
            raise ValueError(f"Unknown body: {body_name}")

        return self.data.xpos[body_id].copy()