"""High-level command coordinator for MotionCore's HTTP API."""

from __future__ import annotations

import os
import math
import threading
from typing import Any

from backend.rl_adapter import RLLocomotion
from backend.simulation_runtime import MotionCoreSimulation
from backend.telemetry import RobotTelemetry


class RobotController:
    VALID_ACTIONS = {"WALK", "STOP", "STAND", "WAVE", "SALUTE"}

    def __init__(
        self,
        simulation: MotionCoreSimulation | None = None,
        rl: RLLocomotion | None = None,
        telemetry_monitor: RobotTelemetry | None = None,
        walk_mode: str | None = None,
    ) -> None:
        self.simulation = simulation or MotionCoreSimulation()
        self.rl = rl or RLLocomotion()
        self.telemetry_monitor = telemetry_monitor or RobotTelemetry()
        self.walk_mode = (walk_mode or os.getenv("MOTIONCORE_WALK_MODE", "scripted")).lower()
        if self.walk_mode not in {"scripted", "rl", "auto"}:
            self.walk_mode = "scripted"

        self._lock = threading.RLock()
        self.state: dict[str, Any] = {
            "status": "IDLE",
            "motion": "STAND",
            "speed": 0.0,
            "controller": "NONE",
            "connected": bool(self.simulation.available or self.rl.readiness()["ready"]),
            "walkMode": self.walk_mode,
            "message": "MotionCore controller ready",
        }

    @staticmethod
    def _validate_speed(speed: float) -> float:
        value = float(speed)
        if not math.isfinite(value) or value < 0.0 or value > 1.0:
            raise ValueError("speed must be between 0.0 and 1.0 m/s")
        return value

    def _simulation_fallen(self) -> bool:
        telemetry = self.simulation.get_telemetry()
        return bool(telemetry.get("available") and telemetry.get("fallen"))

    def _select_walk_controller(self) -> str:
        if self.walk_mode == "scripted":
            return "scripted"
        if self.walk_mode == "rl":
            return "rl"
        return "rl" if self.rl.readiness()["ready"] else "scripted"

    def execute_action(self, action: str, speed: float = 0.0) -> dict[str, Any]:
        action = action.strip().upper()
        if action not in self.VALID_ACTIONS:
            return {
                "success": False,
                "message": f"Unknown action '{action}'",
                "allowed": sorted(self.VALID_ACTIONS),
            }

        try:
            speed = self._validate_speed(speed)
        except (TypeError, ValueError) as exc:
            return {"success": False, "message": str(exc)}

        with self._lock:
            if action == "STOP":
                return self._stop_all()

            if action in {"WALK", "WAVE", "SALUTE"} and self._simulation_fallen():
                return {
                    "success": False,
                    "message": "Fall detected. Send STAND to reset the simulation before another motion.",
                }

            if action == "WALK":
                return self._start_walk(speed)

            if self.rl.is_running():
                return {
                    "success": False,
                    "message": "Stop the Unitree RL controller before starting a MotionCore pose/demo.",
                }

            result = self.simulation.execute_action(action, speed)
            if result.get("success"):
                self.state.update(
                    status="MOVING" if action != "STAND" else "STANDING",
                    motion=action,
                    speed=0.0,
                    controller="MOTIONCORE_MUJOCO",
                    connected=True,
                    message=result.get("message", "Command accepted"),
                )
            else:
                self.state["message"] = result.get("message", "Command failed")
            return result

    def _start_walk(self, speed: float) -> dict[str, Any]:
        if self.rl.is_running():
            return {"success": False, "message": "Robot is already walking with Unitree RL"}

        sim_status = self.simulation.get_status()
        if sim_status.get("busy"):
            return {
                "success": False,
                "message": f"Stop the current simulation motion ({sim_status.get('state')}) first",
            }

        selected = self._select_walk_controller()
        if selected == "rl":
            result = self.rl.start_walk(speed)
            if result.get("success"):
                self.state.update(
                    status=result.get("status", "STARTING"),
                    motion="RL_WALK",
                    speed=speed,
                    controller="UNITREE_RL",
                    connected=True,
                    message=result.get("message", "RL walk started"),
                )
            return result

        result = self.simulation.execute_action("SCRIPTED_WALK", speed)
        if result.get("success"):
            self.state.update(
                status="WALKING",
                motion="SLOW_FORWARD_WALK_V1",
                speed=speed,
                controller="MOTIONCORE_MUJOCO",
                connected=True,
                message=(
                    "Balanced scripted walk started: six steps with foot lift and weight transfer. "
                    "The requested value controls pace, not measured m/s."
                ),
            )
            result["message"] = self.state["message"]
        return result

    def _stop_all(self) -> dict[str, Any]:
        rl_result = self.rl.stop_walk()
        sim_result = self.simulation.stop_motion()
        rl_stopping = bool(rl_result.get("stopping"))
        stopping = rl_stopping or bool(sim_result.get("stopping"))
        self.state.update(
            status="STOPPING" if stopping else "STOPPED",
            motion="STOP",
            speed=0.0,
            controller="UNITREE_RL" if rl_stopping else ("MOTIONCORE_MUJOCO" if stopping else "NONE"),
            connected=bool(self.simulation.available or self.rl.readiness()["ready"]),
            message=rl_result["message"] if rl_stopping else (sim_result["message"] if stopping else "All MotionCore controllers stopped"),
        )
        return {
            "success": bool(rl_result.get("success") and sim_result.get("success")),
            "message": self.state["message"],
            "rl": rl_result,
            "simulation": sim_result,
        }

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            sim_status = self.simulation.get_status()
            rl_status = self.rl.status()

            if self.state["controller"] == "UNITREE_RL":
                rl_state = rl_status.get("status", "WALKING" if rl_status["running"] else "IDLE")
                self.state.update(
                    status=rl_state,
                    motion="RL_WALK" if rl_state in {"STARTING", "WALKING"} else "STOP",
                    speed=rl_status.get("speed", self.state["speed"]) if rl_state in {"STARTING", "WALKING"} else 0.,
                    message=rl_status.get("message") or "RL simulation status updated",
                )

            if self.state["controller"] == "MOTIONCORE_MUJOCO":
                if sim_status.get("fallen"):
                    self.state.update(
                        status="FALLEN",
                        motion="STOP",
                        speed=0.0,
                        message="Fall detected by MotionCore stability monitor",
                    )
                elif not sim_status.get("busy") and self.state["status"] in {"MOVING", "STANDING", "WALKING", "STOPPING"}:
                    was_stopping = self.state["status"] == "STOPPING"
                    self.state.update(
                        status="STOPPED" if was_stopping else "IDLE",
                        motion="STOP" if was_stopping else "STAND",
                        speed=0.0,
                        controller="NONE",
                        message="Walk stopped with both feet down" if was_stopping else "Motion completed",
                    )

            self.state["connected"] = bool(
                self.simulation.available or rl_status["ready"]
            )
            return {
                **self.state,
                "simulationState": sim_status.get("state"),
                "simulationAvailable": bool(sim_status.get("available")),
                "rlReady": bool(rl_status.get("ready")),
                "rlRunning": bool(rl_status.get("running")),
                "speedApplied": rl_status.get("appliedSeq", 0) >= rl_status.get("commandSeq", 1)
                    if self.state["controller"] == "UNITREE_RL" else True,
            }

    def change_speed(self, speed: float) -> dict[str, Any]:
        try:
            speed = self._validate_speed(speed)
        except (TypeError, ValueError) as exc:
            return {"success": False, "message": str(exc)}

        with self._lock:
            if self.rl.is_running():
                if speed <= 0:
                    return {**self._stop_all(), "speed": 0.0, "applied": False}
                result = self.rl.change_speed(speed)
                self.state["message"] = result.get("message", "RL speed update")
                if result.get("success"):
                    self.state["speed"] = speed
                return result
            self.state["speed"] = speed
            if speed <= 0 and self.state["status"] == "WALKING":
                return {**self._stop_all(), "speed": 0.0, "applied": True}
            self.simulation.set_walk_speed(speed)
            return {
                "success": True,
                "speed": speed,
                "applied": True,
                "message": "Scripted walking pace updated; timing changes at the next step.",
            }

    def get_telemetry(self) -> dict[str, Any]:
        with self._lock:
            if self.rl.is_running() or self.state["controller"] == "UNITREE_RL":
                return self.rl.get_telemetry()
            return self.simulation.get_telemetry()

    def shutdown(self) -> None:
        with self._lock:
            try:
                self.rl.shutdown()
            finally:
                self.simulation.shutdown()
