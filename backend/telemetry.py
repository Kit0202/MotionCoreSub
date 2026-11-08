"""Telemetry reader used when an external RL process publishes a JSON sample."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RobotTelemetry:
    def __init__(self, file_path: str | Path | None = None, max_age_seconds: float = 3.0):
        configured = file_path or os.getenv(
            "MOTIONCORE_TELEMETRY_FILE",
            PROJECT_ROOT / "backend" / "robot_telemetry.json",
        )
        self.file = Path(configured).expanduser()
        if not self.file.is_absolute():
            self.file = PROJECT_ROOT / self.file
        self.max_age_seconds = max_age_seconds

    @staticmethod
    def _fallback(message: str | None = None) -> dict[str, Any]:
        data: dict[str, Any] = {
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "leftFoot": False,
            "rightFoot": False,
            "fallen": False,
            "available": False,
            "source": "file",
        }
        if message:
            data["error"] = message
        return data

    def get_data(self) -> dict[str, Any]:
        try:
            stat = self.file.stat()
            age = max(0.0, time.time() - stat.st_mtime)
            if age > self.max_age_seconds:
                return self._fallback(
                    f"Telemetry sample is stale ({age:.1f}s old)"
                )

            with self.file.open("r", encoding="utf-8") as handle:
                data = json.load(handle)

            required = {
                "height": (int, float),
                "roll": (int, float),
                "pitch": (int, float),
                "leftFoot": bool,
                "rightFoot": bool,
                "fallen": bool,
            }
            for key, expected in required.items():
                if key not in data or not isinstance(data[key], expected):
                    return self._fallback(f"Invalid telemetry field: {key}")

            return {
                **data,
                "height": float(data["height"]),
                "roll": float(data["roll"]),
                "pitch": float(data["pitch"]),
                "available": float(data["height"]) > 0,
                "source": "file",
                "ageSeconds": round(age, 3),
            }
        except FileNotFoundError:
            return self._fallback(f"Telemetry file not found: {self.file}")
        except (OSError, json.JSONDecodeError) as exc:
            return self._fallback(f"Unable to read telemetry: {exc}")
