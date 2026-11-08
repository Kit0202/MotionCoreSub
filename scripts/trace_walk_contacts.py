import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


STATE_TO_TEST = "SLOW_FORWARD_WALK_V1"
MAX_STEPS = 25000


def main():
    robot = G1Robot()
    robot.reset_to_keyframe("stand")

    # Let the robot settle before walking
    for _ in range(500):
        robot.step()

    print(f"\nTesting contact sequence for: {STATE_TO_TEST}")
    print("=" * 80)

    robot.state_machine.set_state(STATE_TO_TEST)

    previous_contacts = None

    for step in range(MAX_STEPS):
        robot.state_machine.update()
        robot.step()

        contacts = robot.foot_contacts.get_contact_status()
        stability = robot.stability.get_stability_status()

        current_contacts = (
            contacts["left_contact"],
            contacts["right_contact"]
        )

        # Print when contact status changes
        if current_contacts != previous_contacts:
            print(
                f"Step {step:<6} | "
                f"Left: {str(contacts['left_contact']):<5} "
                f"{contacts['left_force']:>8.2f} N | "
                f"Right: {str(contacts['right_contact']):<5} "
                f"{contacts['right_force']:>8.2f} N | "
                f"Roll: {stability['roll']:>7.3f} | "
                f"Pitch: {stability['pitch']:>7.3f}"
            )

            previous_contacts = current_contacts

        if stability["fallen"]:
            print("\nRobot fell. Contact test stopped.")
            break

        if robot.state_machine.is_idle():
            print("\nWalking motion completed.")
            break

    else:
        print("\nMaximum step limit reached.")

    final_position = robot.get_body_position("pelvis")

    print("\nFinal Status")
    print("=" * 80)
    print(f"Pelvis X     : {final_position[0]:.4f} m")
    print(f"Pelvis Y     : {final_position[1]:.4f} m")
    print(f"Pelvis height: {final_position[2]:.4f} m")
    print(f"Fallen       : {robot.stability.is_fallen()}")


if __name__ == "__main__":
    main()