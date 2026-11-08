import argparse
import csv
import sys
from pathlib import Path

# Add MotionCore project root to Python path
PROJECT_ROOT = Path(__file__).resolve().parents[1]

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motioncore.robot import G1Robot


def evaluate_locomotion(state_name, max_steps=20000):
    robot = G1Robot()

    available_states = robot.state_machine.get_available_states()

    if state_name not in available_states:
        print(f"Unknown state: {state_name}")
        print("Available states:")
        for state in available_states:
            print(f"- {state}")
        return

    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)

    output_file = output_dir / f"{state_name.lower()}_evaluation.csv"

    robot.state_machine.set_state(state_name)

    start_position = None
    end_position = None

    max_roll = 0.0
    max_pitch = 0.0
    fallen = False

    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "step",
            "pelvis_x",
            "pelvis_y",
            "pelvis_z",
            "roll",
            "pitch",
            "fallen"
        ])

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

            writer.writerow([
                step,
                round(float(position[0]), 4),
                round(float(position[1]), 4),
                round(float(position[2]), 4),
                status["roll"],
                status["pitch"],
                status["fallen"]
            ])

            if status["fallen"]:
                fallen = True
                break

            if robot.state_machine.is_idle():
                break

    if start_position is None or end_position is None:
        print("No simulation data collected.")
        return

    x_displacement = float(end_position[0] - start_position[0])
    y_displacement = float(end_position[1] - start_position[1])

    horizontal_displacement = (x_displacement ** 2 + y_displacement ** 2) ** 0.5

    print("\nLocomotion Evaluation Summary")
    print("=" * 60)
    print(f"State                  : {state_name}")
    print(f"X displacement         : {x_displacement:.4f} m")
    print(f"Y displacement         : {y_displacement:.4f} m")
    print(f"Horizontal displacement: {horizontal_displacement:.4f} m")
    print(f"Start height           : {start_position[2]:.4f} m")
    print(f"End height             : {end_position[2]:.4f} m")
    print(f"Max roll               : {max_roll:.4f} rad")
    print(f"Max pitch              : {max_pitch:.4f} rad")
    print(f"Fallen                 : {fallen}")
    print(f"CSV saved to           : {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate MotionCore locomotion stability and displacement"
    )

    parser.add_argument(
        "state",
        nargs="?",
        default="SLOW_WALK_V2_SAFE",
        help="MotionCore locomotion state to evaluate"
    )

    args = parser.parse_args()

    evaluate_locomotion(args.state)


if __name__ == "__main__":
    main()