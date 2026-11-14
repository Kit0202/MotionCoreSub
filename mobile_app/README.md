# MotionCore Mobile

Flutter dashboard for the MotionCore Unitree G1 research prototype. The app communicates with the FastAPI backend over HTTP and displays live simulator telemetry.

## Functions

- Configure the backend server URL at runtime.
- View backend connection state and active controller.
- View pelvis height, roll, pitch and fall status.
- View left/right foot contact.
- Send `STAND`, `WAVE`, `SALUTE` and `WALK` commands.
- Send STOP at any time, including while another command is pending.

In scripted mode, gestures use the MotionCore state machine. In RL mode, walking and gestures use one persistent unified MuJoCo runtime. `auto` tries unified RL at startup and falls back to scripted mode if initialization fails.

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

Height is displayed in metres; roll and pitch are radians. Foot-contact values come from MotionCore's MuJoCo contact detector while the built-in simulator is active. In unified RL mode, telemetry comes directly from the shared MuJoCo simulation. The older subprocess adapter uses session JSON files only when invoked directly. Stale or unavailable samples are deliberately shown as unavailable rather than being treated as stable values.

## Four-can camera pickup

Start `RUN_PICKUP.bat` at the project root after installing
`requirements-vision.txt`. All four free-body cans appear on one table.
Choose **Can to pick up**; selection changes only the requested type, never the
scene objects. **Find selected can** shows all recognised cans and the selected
match. **Detect & pick up** takes a fresh scan and reaches to the matching
camera-estimated position. Place can, then choose another type without resetting.
Stop pauses while retaining the grip; Continue resumes. Reset scene explicitly
restores all cans. Missing/uncertain/duplicate targets block pickup.

The image is the last annotated camera snapshot, not live video. Recognition
is trained on four stylized simulation designs, not real-world brands. The arm
is chosen from the detected object's side. See `../CAN_PICKUP.md` for scope,
API details and backend/Flutter validation commands.
