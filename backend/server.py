from __future__ import annotations

from contextlib import asynccontextmanager
from typing import Literal

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from backend.robot_controller import RobotController


def create_controller():
    import os
    mode = os.getenv("MOTIONCORE_WALK_MODE", "scripted").lower()
    if mode in {"rl", "auto"}:
        try:
            from backend.unified_controller import UnifiedController
            return UnifiedController()
        except Exception:
            if mode == "rl":
                raise
    return RobotController(walk_mode="scripted")


controller = create_controller()


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.controller = controller
    yield
    controller.shutdown()


app = FastAPI(
    title="MotionCore Robot API",
    version="1.0.0",
    lifespan=lifespan,
)

# Useful for Flutter Web during local FYP demonstrations. Native Android/iOS
# clients are not governed by browser CORS, but allowing local browser access
# makes the same backend easier to test.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)


class RobotCommand(BaseModel):
    action: Literal["WALK", "STOP", "STAND", "WAVE", "SALUTE"]
    speed: float = Field(default=0.0, ge=0.0, le=1.0)


class SpeedCommand(BaseModel):
    speed: float = Field(ge=0.0, le=1.0)


def get_controller(request: Request) -> RobotController:
    return getattr(request.app.state, "controller", controller)


@app.get("/")
def home():
    return {
        "message": "MotionCore Backend Running",
        "api": "1.0.0",
        "endpoints": [
            "/robot/status",
            "/robot/telemetry",
            "/robot/action",
            "/robot/stop",
            "/robot/speed",
        ],
    }


@app.get("/health")
def health(request: Request):
    status = get_controller(request).get_status()
    return {
        "ok": bool(status["connected"]),
        "status": status["status"],
        "simulationAvailable": status["simulationAvailable"],
        "rlReady": status["rlReady"],
    }


@app.get("/robot/status")
def status(request: Request):
    return get_controller(request).get_status()


@app.post("/robot/action")
def action(command: RobotCommand, request: Request):
    return get_controller(request).execute_action(command.action, command.speed)


@app.post("/robot/stop")
def stop(request: Request):
    return get_controller(request).execute_action("STOP")


@app.post("/robot/speed")
def speed(command: SpeedCommand, request: Request):
    return get_controller(request).change_speed(command.speed)


@app.get("/robot/telemetry")
def telemetry(request: Request):
    return get_controller(request).get_telemetry()
