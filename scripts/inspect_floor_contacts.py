import sys
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


def get_name(model, object_type, object_id):
    name = mujoco.mj_id2name(model, object_type, object_id)
    return name if name is not None else "<unnamed>"


def main():
    robot = G1Robot()
    robot.reset_to_keyframe("stand")

    # Allow the robot to settle onto the floor
    for _ in range(1000):
        robot.step()

    floor_geom_id = mujoco.mj_name2id(
        robot.model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "floor"
    )

    if floor_geom_id == -1:
        print("Floor geom was not found.")
        return

    floor_contacts = {}

    for contact_index in range(robot.data.ncon):
        contact = robot.data.contact[contact_index]

        if contact.geom1 == floor_geom_id:
            robot_geom_id = contact.geom2
        elif contact.geom2 == floor_geom_id:
            robot_geom_id = contact.geom1
        else:
            continue

        body_id = int(robot.model.geom_bodyid[robot_geom_id])

        geom_name = get_name(
            robot.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            robot_geom_id
        )

        body_name = get_name(
            robot.model,
            mujoco.mjtObj.mjOBJ_BODY,
            body_id
        )

        floor_contacts[robot_geom_id] = {
            "geom_name": geom_name,
            "body_id": body_id,
            "body_name": body_name
        }

    print("\nRobot geoms touching the floor")
    print("=" * 60)

    if not floor_contacts:
        print("No floor contacts were detected.")
        print(f"Total active contacts: {robot.data.ncon}")
        return

    for geom_id, information in floor_contacts.items():
        print(
            f"Geom ID: {geom_id:<3} "
            f"Geom: {information['geom_name']:<12} "
            f"Body ID: {information['body_id']:<3} "
            f"Body: {information['body_name']}"
        )

    print(f"\nNumber of floor-contacting geoms: {len(floor_contacts)}")


if __name__ == "__main__":
    main()