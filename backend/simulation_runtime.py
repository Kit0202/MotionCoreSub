"""Background MuJoCo runtime used by the FastAPI backend.

The runtime deliberately keeps simulator ownership in one thread. FastAPI request
handlers only ask the runtime to start/cancel MotionCore states and read a cached
telemetry snapshot, which avoids stepping MuJoCo concurrently from HTTP threads.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any


class MotionCoreSimulation:
    """Own a :class:`G1Robot` and continuously advance its MuJoCo simulation."""

    ACTION_TO_STATE = {
        "STAND": "STAND",
        "WAVE": "WAVE_LEFT_HAND",
        "SALUTE": "SALUTE_RIGHT",
        "SCRIPTED_WALK": "SLOW_FORWARD_WALK_V1",
    }

    def __init__(
        self,
        robot: Any | None = None,
        *,
        auto_start: bool = True,
        viewer: bool | None = None,
    ) -> None:
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.robot = robot
        self.available = False
        self.error: str | None = None
        self.last_action = "IDLE"
        self.requested_speed = 0.0
        self._telemetry: dict[str, Any] = self._empty_telemetry()
        self.viewer_enabled = (
            viewer
            if viewer is not None
            else os.getenv("MOTIONCORE_VIEWER", "0").strip().lower()
            in {"1", "true", "yes", "on"}
        )

        if self.robot is None:
            try:
                from motioncore.robot import G1Robot

                self.robot = G1Robot()
            except Exception as exc:  # pragma: no cover - environment dependent
                self.error = f"MotionCore simulation unavailable: {exc}"
                print(self.error)
                self.robot = None

        if self.robot is not None:
            try:
                # Always begin from a known, stable pose for API-controlled demos.
                self.robot.reset_to_keyframe("stand")
                self.available = True
                self._capture_telemetry_locked()
            except Exception as exc:  # pragma: no cover - depends on local MuJoCo
                self.error = f"MuJoCo initialization failed: {exc}"
                self.available = False

        if auto_start and self.available:
            self.start()

    @staticmethod
    def _empty_telemetry() -> dict[str, Any]:
        return {
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "leftFoot": False,
            "rightFoot": False,
            "leftForce": 0.0,
            "rightForce": 0.0,
            "fallen": False,
            "available": False,
            "source": "mujoco",
        }

    def start(self) -> None:
        if not self.available:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="motioncore-mujoco-runtime",
            daemon=True,
        )
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

    def _run(self) -> None:  # pragma: no cover - continuous simulator loop
        viewer = None
        try:
            if self.viewer_enabled:
                try:
                    import mujoco.viewer

                    viewer = mujoco.viewer.launch_passive(
                        self.robot.model,
                        self.robot.data,
                    )
                except Exception as exc:
                    self.error = f"Viewer could not be opened: {exc}"
                    viewer = None

            timestep = float(getattr(self.robot.model.opt, "timestep", 0.002))
            timestep = max(0.0005, min(timestep, 0.02))
            next_tick = time.perf_counter()
            next_viewer_sync = next_tick

            while not self._stop_event.is_set():
                with self._lock:
                    self.robot.state_machine.update()
                    self.robot.step()
                    self._capture_telemetry_locked()

                    if self._telemetry["fallen"] and not self.robot.state_machine.is_idle():
                        self.robot.state_machine.cancel("fall detected")
                        self.last_action = "FALLEN"

                    now = time.perf_counter()
                    if viewer is not None and viewer.is_running() and now >= next_viewer_sync:
                        viewer.sync()
                        next_viewer_sync = now + 1.0 / 30.0

                next_tick += timestep
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
        except Exception as exc:
            self.error = f"Simulation runtime stopped: {exc}"
            self.available = False
        finally:
            if viewer is not None:
                try:
                    viewer.close()
                except Exception:
                    pass

    def _capture_telemetry_locked(self) -> None:
        try:
            stability = self.robot.stability.get_stability_status()
            contacts = self.robot.foot_contacts.get_contact_status()
            self._telemetry = {
                "height": float(stability["height"]),
                "roll": float(stability["roll"]),
                "pitch": float(stability["pitch"]),
                "leftFoot": bool(contacts["left_contact"]),
                "rightFoot": bool(contacts["right_contact"]),
                "leftForce": float(contacts["left_force"]),
                "rightForce": float(contacts["right_force"]),
                "fallen": bool(stability["fallen"]),
                "available": True,
                "source": "mujoco",
                "timestamp": time.time(),
            }
        except Exception as exc:
            self.error = f"Telemetry update failed: {exc}"
            self._telemetry = self._empty_telemetry()

    def execute_action(self, action: str, speed: float = 0.0) -> dict[str, Any]:
        if not self.available or self.robot is None:
            return {
                "success": False,
                "message": self.error or "MuJoCo simulation is unavailable",
            }

        action = action.upper()
        state_name = self.ACTION_TO_STATE.get(action)
        if state_name is None:
            return {"success": False, "message": f"Unsupported simulation action: {action}"}

        with self._lock:
            if self._telemetry.get("fallen") and action not in {"STAND"}:
                return {
                    "success": False,
                    "message": "Robot is marked as fallen. Use STAND to reset the simulation first.",
                }

            if not self.robot.state_machine.is_idle():
                return {
                    "success": False,
                    "message": f"Robot is busy with {self.robot.state_machine.current_state}",
                }

            # STAND doubles as a safe simulation recovery/reset command.
            if action == "STAND" and self._telemetry.get("fallen"):
                self.robot.reset_to_keyframe("stand")
                self._capture_telemetry_locked()

            if action == "SCRIPTED_WALK":
                if speed <= 0:
                    return {"success": False, "message": "Set walking pace above zero, then press Walk."}
                self.robot.motion.walk_speed = float(speed)
            self.requested_speed = float(speed)
            self.robot.state_machine.set_state(state_name)
            self.last_action = action
            return {
                "success": True,
                "message": f"{action.replace('_', ' ').title()} started in MuJoCo",
                "state": state_name,
                "controller": "MOTIONCORE_MUJOCO",
                "speed": self.requested_speed,
            }

    def stop_motion(self, reason: str = "stop requested") -> dict[str, Any]:
        if not self.available or self.robot is None:
            return {"success": True, "message": "No MuJoCo motion was running"}
        with self._lock:
            previous = self.robot.state_machine.current_state
            walker = getattr(self.robot.motion, "active_walk", None)
            if walker is not None and not self._telemetry.get("fallen"):
                walker.stop_requested = True
                self.last_action = "STOPPING"
                self.requested_speed = 0.0
                return {
                    "success": True, "stopping": True,
                    "message": "Stopping: finishing the current phase, landing and settling before stopping.",
                    "previousState": previous,
                }
            self.robot.state_machine.cancel(reason)
            self.last_action = "STOP"
            self.requested_speed = 0.0
            return {
                "success": True,
                "message": "MotionCore simulation stopped",
                "previousState": previous,
            }

    def set_walk_speed(self, speed: float) -> None:
        with self._lock:
            self.requested_speed = float(speed)
            if self.robot is not None:
                self.robot.motion.walk_speed = float(speed)
                walker = getattr(self.robot.motion, "active_walk", None)
                if walker is not None:
                    if speed <= 0:
                        walker.stop_requested = True
                    else:
                        walker.speed = float(speed)

    def get_telemetry(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._telemetry)

    def get_status(self) -> dict[str, Any]:
        if not self.available or self.robot is None:
            return {
                "available": False,
                "busy": False,
                "state": "UNAVAILABLE",
                "fallen": False,
                "error": self.error,
            }
        with self._lock:
            state = self.robot.state_machine.current_state
            fallen = bool(self._telemetry.get("fallen", False))
            return {
                "available": True,
                "busy": not self.robot.state_machine.is_idle(),
                "state": "FALLEN" if fallen else state,
                "fallen": fallen,
                "lastAction": self.last_action,
                "error": self.error,
            }
