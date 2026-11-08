import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


STATES_TO_EVALUATE = [
    "WEIGHT_SHIFT_TEST",
    "KNEE_BEND_TEST",
    "TINY_FOOT_LIFT_TEST",
    "FORWARD_STEP_TEST",
    "SLOW_WALK_V2_SAFE"
]


def evaluate_state(state_name, max_steps=25000):
    robot = G1Robot()

    robot.state_machine.set_state(state_name)

    start_position = None
    end_position = None

    max_roll = 0.0
    max_pitch = 0.0
    fallen = False
    steps_executed = 0

    for step in range(max_steps):
        robot.state_machine.update()
        robot.step()

        position = robot.get_body_position("pelvis")
        status = robot.stability.get_stability_status()

        if start_position is None:
            start_position = position.copy()

        end_position = position.copy()

        max_roll = max(max_roll, abs(status["roll"]))
        max_pitch = max(max_pitch, abs(status["pitch"]))

        steps_executed = step + 1

        if status["fallen"]:
            fallen = True
            break

        if robot.state_machine.is_idle():
            break

    if start_position is None or end_position is None:
        return None

    x_displacement = float(end_position[0] - start_position[0])
    y_displacement = float(end_position[1] - start_position[1])
    horizontal_displacement = (x_displacement ** 2 + y_displacement ** 2) ** 0.5

    return {
        "state": state_name,
        "steps_executed": steps_executed,
        "x_displacement": round(x_displacement, 4),
        "y_displacement": round(y_displacement, 4),
        "horizontal_displacement": round(horizontal_displacement, 4),
        "start_height": round(float(start_position[2]), 4),
        "end_height": round(float(end_position[2]), 4),
        "max_roll": round(max_roll, 4),
        "max_pitch": round(max_pitch, 4),
        "fallen": fallen
    }


def main():
    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)

    output_file = output_dir / "locomotion_comparison.csv"

    results = []

    print("\nMotionCore Locomotion Comparison")
    print("=" * 80)

    for state in STATES_TO_EVALUATE:
        print(f"Evaluating: {state}")

        result = evaluate_state(state)

        if result is not None:
            results.append(result)

    with open(output_file, mode="w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "state",
                "steps_executed",
                "x_displacement",
                "y_displacement",
                "horizontal_displacement",
                "start_height",
                "end_height",
                "max_roll",
                "max_pitch",
                "fallen"
            ]
        )

        writer.writeheader()
        writer.writerows(results)

    print("\nSummary")
    print("=" * 80)

    for result in results:
        print(
            f"{result['state']:22} "
            f"Fallen: {str(result['fallen']):5} "
            f"X: {result['x_displacement']:8.4f} "
            f"Y: {result['y_displacement']:8.4f} "
            f"H: {result['horizontal_displacement']:8.4f} "
            f"Roll: {result['max_roll']:6.4f} "
            f"Pitch: {result['max_pitch']:6.4f}"
        )

    print(f"\nComparison saved to: {output_file}")


if __name__ == "__main__":
    main()