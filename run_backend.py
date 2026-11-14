"""Convenient launcher for the MotionCore FastAPI backend."""

from __future__ import annotations

import argparse
import os

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MotionCore backend")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open the MuJoCo viewer for mobile/API controlled motions",
    )
    parser.add_argument(
        "--walk-mode",
        choices=["scripted", "rl", "auto"],
        default=os.getenv("MOTIONCORE_WALK_MODE", "scripted"),
        help="Walking backend: scripted MuJoCo, unified pretrained RL, or auto with scripted fallback",
    )
    parser.add_argument(
        "--scene", choices=["motion", "pickup"],
        default=os.getenv("MOTIONCORE_SCENE", "motion"),
        help="Scene: walking/gestures or articulated-hand can pickup",
    )
    args = parser.parse_args()

    if args.viewer:
        os.environ["MOTIONCORE_VIEWER"] = "1"
    os.environ["MOTIONCORE_WALK_MODE"] = args.walk_mode
    os.environ["MOTIONCORE_SCENE"] = args.scene

    uvicorn.run("backend.server:app", host=args.host, port=args.port, reload=False)


if __name__ == "__main__":
    main()
