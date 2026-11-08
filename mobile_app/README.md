# MotionCore Mobile

Flutter dashboard for the MotionCore Unitree G1 research prototype. The app communicates with the FastAPI backend over HTTP and displays live simulator telemetry.

## Functions

- Configure the backend server URL at runtime.
- View backend connection state and active controller.
- View pelvis height, roll, pitch and fall status.
- View left/right foot contact.
- Send `STAND`, `WAVE`, `SALUTE` and `WALK` commands.
- Send STOP at any time, including while another command is pending.

`WAVE`, `SALUTE`, and `STAND` now execute the MotionCore state machine in the backend MuJoCo runtime. `WALK` uses the backend-selected walking mode (`scripted`, `rl`, or `auto`).

## Run

Start the backend first from the project root:

```powershell
python run_backend.py --viewer --walk-mode scripted
```

Then:

```powershell
cd mobile_app
flutter pub get
flutter run
```

Use the settings icon in the app to enter the laptop's FastAPI address, for example `http://192.168.1.10:8000`. A physical phone must be on the same reachable Wi-Fi/LAN as the laptop. Android local clear-text HTTP is enabled for this FYP development/demo build.

A build-time default can also be supplied:

```powershell
flutter run --dart-define=ROBOT_API_URL=http://192.168.1.10:8000
```

## Telemetry Notes

Height is displayed in metres; roll and pitch are radians. Foot-contact values come from MotionCore's MuJoCo contact detector while the built-in simulator is active. During external RL mode, telemetry is read from the configured JSON publisher file. Stale or unavailable samples are deliberately shown as unavailable rather than being treated as stable values.
