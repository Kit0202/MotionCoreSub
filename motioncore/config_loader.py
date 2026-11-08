import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ConfigLoader:
    """Loads MotionCore configuration files independent of the current working directory."""

    def __init__(self, config_path="config/motion_config.json"):
        path = Path(config_path).expanduser()
        self.config_path = path if path.is_absolute() else PROJECT_ROOT / path

    def load_motion_config(self):
        if not self.config_path.exists():
            raise FileNotFoundError(
                f"Motion configuration file not found: {self.config_path}"
            )

        with self.config_path.open("r", encoding="utf-8") as file:
            return json.load(file)
