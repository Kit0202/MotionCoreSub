import mujoco
import numpy as np


class FootContactDetector:
    """Detects whether the Unitree G1's feet are touching the floor."""

    def __init__(self, robot):
        self.robot = robot
        self.model = robot.model
        self.data = robot.data

        self.floor_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "floor"
        )

        if self.floor_geom_id == -1:
            raise ValueError("Floor geom named 'floor' was not found.")

        self.left_foot_body_id = self._find_foot_body("left")
        self.right_foot_body_id = self._find_foot_body("right")

    def _find_foot_body(self, side):
        expected_name = f"{side}_ankle_roll_link"

        body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            expected_name
        )

        if body_id != -1:
            return body_id

        # Fallback search if the exact body name changes
        for current_body_id in range(self.model.nbody):
            body_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                current_body_id
            )

            if body_name is None:
                continue

            lower_name = body_name.lower()

            if side in lower_name and "ankle" in lower_name:
                return current_body_id

        raise ValueError(f"Could not find the {side} foot body.")

    def get_contact_status(self):
        left_contact = False
        right_contact = False

        left_force = 0.0
        right_force = 0.0

        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]

            if contact.geom1 == self.floor_geom_id:
                robot_geom_id = contact.geom2
            elif contact.geom2 == self.floor_geom_id:
                robot_geom_id = contact.geom1
            else:
                continue

            body_id = int(self.model.geom_bodyid[robot_geom_id])

            contact_force = np.zeros(6)

            mujoco.mj_contactForce(
                self.model,
                self.data,
                contact_index,
                contact_force
            )

            normal_force = max(0.0, float(contact_force[0]))

            if body_id == self.left_foot_body_id:
                left_contact = True
                left_force += normal_force

            elif body_id == self.right_foot_body_id:
                right_contact = True
                right_force += normal_force

        return {
            "left_contact": left_contact,
            "right_contact": right_contact,
            "left_force": round(left_force, 3),
            "right_force": round(right_force, 3)
        }

    def both_feet_contacting(self):
        status = self.get_contact_status()

        return status["left_contact"] and status["right_contact"]