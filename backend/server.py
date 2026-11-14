from __future__ import annotations

from contextlib import asynccontextmanager
import logging
from typing import Literal

from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import Response
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from backend.robot_controller import RobotController

logger = logging.getLogger(__name__)

def create_controller():
    import os
    scene = os.getenv("MOTIONCORE_SCENE", "motion").lower()
    if scene == "pickup":
        from backend.pickup_controller import PickupController
        return PickupController()
    if scene != "motion":
        raise ValueError("MOTIONCORE_SCENE must be motion or pickup")
    mode = os.getenv("MOTIONCORE_WALK_MODE", "scripted").lower()
    if mode in {"rl", "auto"}:
        try:
            from backend.unified_controller import UnifiedController
            return UnifiedController()
        except Exception:
            if mode == "rl":
                raise
            logger.warning("Unified RL initialization failed; using scripted mode", exc_info=True)
    return RobotController(walk_mode="scripted")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Create a fresh runtime for each server lifespan, never at module import.
    controller = create_controller()
    app.state.controller = controller
    try:
        yield
    finally:
        controller.shutdown()
        del app.state.controller


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
    action: Literal["WALK", "STOP", "STAND", "WAVE", "SALUTE", "GRAB", "PLACE", "RESET", "RESUME", "SCAN"]
    speed: float = Field(default=0.0, ge=0.0, le=1.0)


class CanSelectionCommand(BaseModel):
    canType: Literal["cola", "tuna", "100plus", "pepsi"]


class SpeedCommand(BaseModel):
    speed: float = Field(ge=0.0, le=1.0)


def get_controller(request: Request):
    return request.app.state.controller


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
            "/robot/can",
            "/robot/camera",
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


@app.post("/robot/can")
def select_can(command: CanSelectionCommand, request: Request):
    controller = get_controller(request)
    if not hasattr(controller, "select_can"):
        return {"success":False, "message":"Can selection is available in the pickup scene."}
    return controller.select_can(command.canType)


@app.get('/robot/camera')
def camera(request: Request):
    controller = get_controller(request)
    jpeg = controller.get_camera_image() if hasattr(controller,'get_camera_image') else None
    if jpeg is None:
        raise HTTPException(status_code=404,detail='No camera snapshot. Scan a can first.')
    return Response(content=jpeg,media_type='image/jpeg',headers={'Cache-Control':'no-store'})
