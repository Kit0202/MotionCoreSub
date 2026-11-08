"""Launch the optional external Unitree RL controller using MotionCore settings."""

from backend.rl_adapter import RLLocomotion


def main():
    rl = RLLocomotion()
    ready = rl.readiness()
    if not ready["ready"]:
        missing = "\n  - ".join(ready["missing"])
        raise FileNotFoundError(
            "Unitree RL is not configured. Set UNITREE_RL_ROOT or the more "
            f"specific environment variables. Missing:\n  - {missing}"
        )

    result = rl.start_walk(0.5)
    if not result["success"]:
        raise RuntimeError(result["message"])

    print(result["message"])
    print("Press Ctrl+C to stop.")
    try:
        process = rl.process
        if process is not None:
            process.wait()
    except KeyboardInterrupt:
        rl.stop_walk()


if __name__ == "__main__":
    main()
