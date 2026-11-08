# MotionCore: Unitree RL walking integration

## Windows quick start

Extract this ZIP and open the inner MotionCoreSub folder: it contains
SETUP_RL.bat, RUN_RL.bat, run_backend.py and requirements.txt. Preserve your
existing folder as a backup before copying this update over it.

1. Run SETUP_RL.bat once. It creates a Python 3.12 environment if needed and
   installs the Python dependencies plus CPU PyTorch. No Isaac Gym, CUDA, separate
   RL repository installation or training is needed for this supplied policy.
2. Run RUN_RL.bat. This starts the backend in RL mode and opens the original
   MotionCore gesture viewer.
3. In VS Code's second terminal, from this same project folder:

```powershell
cd mobile_app
flutter pub get
flutter run
```

4. In Server connection, enter your laptop's Wi-Fi address. The last supplied
   address was http://192.168.0.93:8000. Phone and laptop must share the Wi-Fi.
5. Set speed to 0.50 and press Start walking. Loading PyTorch can take a few
   seconds: the app shows STARTING, then a second window opens for RL walking.
6. Release the speed slider while walking to send a live speed update. The app
   displays the applied (ramped) command and measured forward speed separately.
7. Press Stop. The status changes to STOPPING while speed ramps down. After low
   translational speed is detected, the RL walking window closes and status is
   STOPPED. It usually takes 2–4 seconds of simulation time. Slow computers can
   take longer in wall-clock time.
8. You can then press Stand, Wave or Salute in the original robot window.

The complete manual backend command is:

```powershell
.\.venv\Scripts\python.exe run_backend.py --viewer --walk-mode rl
```

To keep using the previous slow scripted gait, select --walk-mode scripted.
The default launch mode remains scripted; RUN_RL.bat explicitly selects RL.

## What this update implements

- Bundles the unchanged pretrained G1 policy, its original 12-joint model and
  required meshes, preserving Unitree's license and attribution.
- Applies app speed commands to the policy, with acceleration/deceleration
  limited to 0.5 m/s². Commands are acknowledged by sequence number; the API
  does not equate a queued command with measured velocity.
- Uses genuine Euler roll/pitch in radians and measured forward velocity in m/s.
- Sends foot contacts, contact forces, height and fall state from the RL model.
- Keeps all runtime paths relative to this project. No D:\MotionCore paths.
- Uses separate session folders and atomic JSON writes so old or partial samples
  cannot masquerade as live telemetry. A parent heartbeat prevents an orphaned
  worker from walking indefinitely.
- Reports STARTING, WALKING, STOPPING, STOPPED, FALLEN and ERROR. Duplicate Walk
  and gesture commands are blocked while the RL process is active.
- Reports startup failures rather than claiming that a launched process is
  already walking. Diagnostic logs are under outputs/rl_runtime/<session>/worker.log.
- Closes the RL simulation after a detected fall or deceleration timeout. A new
  Walk starts a fresh policy/simulation session. Use Stand to clear a displayed
  fall state and return to the gesture simulator before retrying.

The policy still steps at a zero command; Stop decelerates and closes its
simulation. It is not a static-standing controller or a hardware emergency stop.

## Simulation scope

This release intentionally uses TWO supplied robot models. RL walking runs in
its own window with the 12-joint G1 model. Stand/Wave/Salute use the existing
MotionCore model. Position is not transferred between them; a new RL session
starts at the origin. The table-transfer scene, hands and object carrying have
not been combined with the RL controller. They require separate validation with
the changed mass distribution and actuator mapping.

Motion comes from the pretrained policy's joint torques and ground contact.
The deployment does not directly move or pin the floating base. Walking is
forward only; observed lateral drift means this is not a navigation controller.
The gait can include brief flight phases, particularly at faster settings.

## Verification

Headless Linux tests used Python 3.12.14, MuJoCo 3.13.0, CPU PyTorch 2.14.0 and
NumPy 2.3.5. Each speed test ran 15 simulated seconds, then 5 seconds at zero.
Steady speed was averaged over simulation seconds 5–14.

| Requested speed | Measured forward speed | Minimum pelvis height | Fall |
|---|---:|---:|---|
| 0.20 m/s | 0.201 m/s | 0.772 m | No |
| 0.50 m/s | 0.476 m/s | 0.765 m | No |
| 1.00 m/s | 0.915 m/s | 0.754 m | No |

Lateral displacement across the full runs was approximately 0.58–1.69 m.
Raw measurements are in outputs/rl_validation.json. All 52 Python regression tests passed. A real subprocess test also
verified startup, a 0.3 → 0.7 m/s live command, telemetry acknowledgment, controlled
Stop, a subsequent gesture command, and starting/stopping a new RL session.

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe scripts\evaluate_rl.py --speed 0.5
```

Flutter and Windows graphical execution were not available in the validation
environment. The Dart changes have source-level review and added tests; run
flutter test from mobile_app on your machine. Close existing backend sessions
before launching this version so port 8000 is available.

## Files changed for RL integration

- backend/rl_adapter.py: subprocess lifecycle, acknowledged commands, heartbeat.
- backend/rl_policy.py: CPU policy inference and physical simulation.
- backend/rl_worker.py: viewer, speed ramp, telemetry and controlled shutdown.
- backend/rl_ipc.py: atomic JSON transport.
- backend/robot_controller.py: truthful RL state and gesture handoff.
- mobile_app/lib/controllers/dashboard_controller.dart: live speed and STARTING.
- mobile_app/lib/services/robot_api.dart: POST /robot/speed.
- mobile_app/lib/models/robot_status.dart and robot_telemetry.dart: status detail,
  measured speed and explicit rejection of unavailable telemetry.
- mobile_app/lib/screens/home_page.dart: applied/measured speed and status detail.
- rl_assets/: required original policy/model files and license.
- requirements-rl.txt, SETUP_RL.bat, RUN_RL.bat: setup and launch.
- scripts/evaluate_rl.py and tests/test_rl_integration.py: repeatable validation.

References used for API behavior:
https://mujoco.readthedocs.io/en/stable/programming/simulation.html
https://docs.pytorch.org/docs/stable/generated/torch.jit.load.html
