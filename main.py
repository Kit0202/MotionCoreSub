import argparse
import mujoco.viewer
import time

from motioncore.robot import G1Robot


def main():
    robot = G1Robot()

    parser = argparse.ArgumentParser(
        description="MotionCore Unitree G1 Demo Runner"
    )

    parser.add_argument(
        "state",
        nargs="?",
        default="DEMO_SALUTE",
        help="MotionCore state/demo to run"
    )

    parser.add_argument(
        "--list",
        action="store_true",
        help="List all available MotionCore states"
    )

    args = parser.parse_args()

    available_states = robot.state_machine.get_available_states()

    if args.list:
        print("\nAvailable MotionCore States:")
        for state in available_states:
            print(f"- {state}")
        return

    if args.state not in available_states:
        print(f"Unknown state: {args.state}")
        print("\nUse this command to see available states:")
        print("python main.py --list")
        return

    print(f"Running MotionCore state: {args.state}")

    robot.state_machine.set_state(args.state)

    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:

        while viewer.is_running():

            robot.state_machine.update()

            robot.step()
            
            status = robot.stability.get_stability_status()

            if status["fallen"]:
                print("Robot has fallen. Stopping simulation.")
                print(status)
                break

            viewer.sync()

            time.sleep(0.002)


if __name__ == "__main__":
    main()