# RL walking update

For the pretrained Unitree walking policy and live mobile controls, read
[RL_SETUP.md](RL_SETUP.md). On Windows, run SETUP_RL.bat once, then RUN_RL.bat.

# MotionCore for Humanoids

MotionCore is a simulation-first humanoid motion-control framework for the Unitree G1. It combines a reusable Python motion engine, MuJoCo simulation, a FastAPI control backend, an optional Unitree RL locomotion adapter, telemetry/fall monitoring, and a Flutter mobile dashboard.

## Implemented Features

- Unitree G1 model loading and joint control in MuJoCo
- Joint-limit validation and smooth single/multi-joint interpolation
- STAND, WAVE and SALUTE motions controlled through the MotionCore state machine
- Scripted locomotion experiments including `SLOW_FORWARD_WALK_V1`
- Stability monitoring using pelvis height, roll and pitch
- Left/right foot contact and contact-force monitoring
- FastAPI backend for status, actions, stop, speed and telemetry
- Background MuJoCo runtime so mobile/API commands execute real MotionCore motions
- Emergency STOP/cancellation and fall-based motion blocking
- Flutter dashboard with server configuration, live telemetry and command controls
- Bundled Unitree RL policy with live speed commands, measured telemetry and controlled stop
- Automated Python and Flutter tests

## Architecture

```text
Flutter Mobile App
        |
        | HTTP / JSON
        v
FastAPI Backend / RobotController
        |
        +-----------------------+
        |                       |
        v                       v
MotionCore MuJoCo Runtime   Unitree RL Adapter (optional)
        |
        v
State Machine -> Motion Sequencer -> Trajectory Engine
        |
        v
Robot Interface -> MuJoCo G1
        |
        +--> Stability + Foot Contact Telemetry
```

The `UnitreeSDK2Interface` remains an explicit future-hardware interface. This repository does **not** claim that physical Unitree G1 control has been tested.

## Quick Start (Windows)

Use Python 3.12 if possible.

See `WALKING_UPDATE.md` for the balanced walking update, measured results,
installation instructions, and Stop behavior.

```powershell
cd MotionCoreSub
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python run_backend.py --viewer --walk-mode scripted
```

Then open another terminal:

```powershell
cd MotionCoreSub\mobile_app
flutter pub get
flutter run
```

In the mobile app, open **Server connection** and enter the laptop's LAN address, for example `http://192.168.1.10:8000`. The backend must be started with `--host 0.0.0.0` (the default in `run_backend.py`) and Windows Firewall must allow Python on the private network.

For an Android emulator, `http://10.0.2.2:8000` usually points to the host PC. For a physical phone, use the laptop's IPv4 address on the same Wi-Fi.

## Backend Commands

- `GET /health` - backend readiness
- `GET /robot/status` - active motion/controller state
- `GET /robot/telemetry` - height, roll, pitch, foot contacts and fall state
- `POST /robot/action` - `WALK`, `STAND`, `WAVE`, `SALUTE`, `STOP`
- `POST /robot/stop` - stop both MotionCore and external RL controllers
- `POST /robot/speed` - update requested walking speed

Example:

```json
{
  "action": "WAVE",
  "speed": 0.0
}
```

## Walking Modes

`scripted` is the submission-safe default because it is self-contained and uses the included MuJoCo model. `rl` launches the bundled Unitree G1 worker in its own simulation window. `auto` uses RL when its dependencies and assets are available and otherwise falls back to scripted walking.

```powershell
python run_backend.py --viewer --walk-mode scripted
python run_backend.py --viewer --walk-mode auto
python run_backend.py --walk-mode rl
```

To use RL, run SETUP_RL.bat once and RUN_RL.bat to launch. The app sends live commands to the worker and displays applied command and measured forward speed. See RL_SETUP.md for behavior, validation, and the two-model limitation.

## Tests

```powershell
python -m pytest
```

MuJoCo-dependent tests run when the `mujoco` package is installed. Pure trajectory, pose and backend-controller tests do not require a graphical simulator.

## Important FYP Scope Statement

The implemented and demonstrable scope is **MuJoCo-based Unitree G1 motion control with a mobile/API interface**. The SDK2 class is intentionally a future integration point and should not be described as completed real-robot control until it has been connected and tested on physical hardware.
