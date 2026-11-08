from datetime import datetime
from pathlib import Path
import csv
import threading


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class MotionLogger:
    """Records MotionCore state execution events."""

    def __init__(self, log_dir="logs", filename="motion_log.csv"):
        log_dir = Path(log_dir).expanduser()
        self.log_dir = log_dir if log_dir.is_absolute() else PROJECT_ROOT / log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / filename
        self._lock = threading.Lock()

        if not self.log_file.exists():
            with self.log_file.open(mode="w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow([
                    "timestamp",
                    "event",
                    "state",
                    "duration_seconds",
                ])

    def log(self, event, state, duration=None):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            with self.log_file.open(mode="a", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, event, state, duration])
        print(f"[LOG] {event}: {state}")
