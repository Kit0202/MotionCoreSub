from backend.robot_controller import RobotController


class FakeSimulation:
    def __init__(self):
        self.available = True
        self.busy = False
        self.fallen = False
        self.last = None

    def get_telemetry(self):
        return {
            "height": 0.79,
            "roll": 0.0,
            "pitch": 0.0,
            "leftFoot": True,
            "rightFoot": True,
            "fallen": self.fallen,
            "available": True,
            "source": "mujoco",
        }

    def get_status(self):
        return {
            "available": True,
            "busy": self.busy,
            "state": self.last or "IDLE",
            "fallen": self.fallen,
            "error": None,
        }

    def execute_action(self, action, speed=0.0):
        if self.busy:
            return {"success": False, "message": "busy"}
        self.last = action
        self.busy = True
        return {"success": True, "message": f"{action} started"}

    def stop_motion(self, reason="stop requested"):
        self.busy = False
        self.last = "STOP"
        return {"success": True, "message": "stopped"}

    def set_walk_speed(self, speed):
        self.speed = speed

    def shutdown(self):
        pass


class FakeRL:
    def __init__(self, ready=False):
        self.ready = ready
        self.running = False

    def readiness(self):
        return {"ready": self.ready, "missing": [] if self.ready else ["rl"]}

    def is_running(self):
        return self.running

    def start_walk(self, speed):
        if not self.ready:
            return {"success": False, "message": "not configured"}
        self.running = True
        return {"success": True, "message": "RL controller launched", "speed": speed}

    def stop_walk(self):
        self.running = False
        return {"success": True, "message": "RL stopped"}

    def status(self):
        return {"ready": self.ready, "running": self.running}

    def get_telemetry(self):
        return {**FakeTelemetry().get_data(), "source": "unitree_rl"}

    def change_speed(self, speed):
        return {"success": True, "speed": speed, "applied": False}

    def shutdown(self):
        self.stop_walk()


class FakeTelemetry:
    def get_data(self):
        return {
            "height": 0.8, "roll": 0.0, "pitch": 0.0,
            "leftFoot": True, "rightFoot": True, "fallen": False,
            "available": True, "source": "file"
        }


def make_controller(*, walk_mode="scripted", rl_ready=False):
    return RobotController(
        simulation=FakeSimulation(),
        rl=FakeRL(rl_ready),
        telemetry_monitor=FakeTelemetry(),
        walk_mode=walk_mode,
    )


def test_wave_executes_real_simulation_action():
    controller = make_controller()
    result = controller.execute_action("WAVE")
    assert result["success"] is True
    assert controller.simulation.last == "WAVE"
    assert controller.get_status()["controller"] == "MOTIONCORE_MUJOCO"


def test_scripted_walk_is_self_contained():
    controller = make_controller(walk_mode="scripted")
    result = controller.execute_action("WALK", 0.5)
    assert result["success"] is True
    assert controller.simulation.last == "SCRIPTED_WALK"
    assert controller.get_status()["status"] == "WALKING"


def test_rl_mode_uses_external_controller_when_configured():
    controller = make_controller(walk_mode="rl", rl_ready=True)
    result = controller.execute_action("WALK", 0.6)
    assert result["success"] is True
    assert controller.rl.running is True
    assert controller.get_status()["controller"] == "UNITREE_RL"
    assert controller.get_telemetry()["source"] == "unitree_rl"


def test_stop_stops_both_backends():
    controller = make_controller(walk_mode="rl", rl_ready=True)
    controller.execute_action("WALK", 0.4)
    result = controller.execute_action("STOP")
    assert result["success"] is True
    assert controller.rl.running is False
    assert controller.simulation.busy is False
    assert controller.get_status()["status"] == "STOPPED"


def test_fall_blocks_motion_but_stand_is_allowed():
    controller = make_controller()
    controller.simulation.fallen = True
    denied = controller.execute_action("WAVE")
    assert denied["success"] is False
    stand = controller.execute_action("STAND")
    assert stand["success"] is True


def test_invalid_speed_rejected():
    controller = make_controller()
    assert controller.execute_action("WALK", 1.5)["success"] is False
