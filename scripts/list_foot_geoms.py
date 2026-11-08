import sys
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


def main():
    robot = G1Robot()

    keywords = ["foot", "ankle", "sole", "toe", "heel"]
    matching_geoms = []

    print("\nPossible foot geoms")
    print("=" * 50)

    for geom_id in range(robot.model.ngeom):
        geom_name = mujoco.mj_id2name(
            robot.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            geom_id
        )

        if geom_name is None:
            continue

        if any(keyword in geom_name.lower() for keyword in keywords):
            matching_geoms.append((geom_id, geom_name))
            print(f"{geom_id}: {geom_name}")

    if not matching_geoms:
        print("No obvious foot geom names were found.")
        print("\nAll named geoms:")
        print("=" * 50)

        for geom_id in range(robot.model.ngeom):
            geom_name = mujoco.mj_id2name(
                robot.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                geom_id
            )

            if geom_name is not None:
                print(f"{geom_id}: {geom_name}")


if __name__ == "__main__":
    main()