import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


def main():
    robot = G1Robot()
    robot.reset_to_keyframe("stand")

    # Allow the robot to settle on the floor
    for _ in range(1000):
        robot.step()

    status = robot.foot_contacts.get_contact_status()

    print("\nFoot Contact Status")
    print("=" * 50)
    print(f"Left foot contact : {status['left_contact']}")
    print(f"Right foot contact: {status['right_contact']}")
    print(f"Left contact force: {status['left_force']}")
    print(f"Right contact force: {status['right_force']}")


if __name__ == "__main__":
    main()