# Balanced walking update

## Install on Windows

1. Stop the Python backend with Ctrl+C. Stop the Flutter app.
2. Keep a backup of your existing folder, then extract this archive. Open the
   MotionCoreSub folder in VS Code (the folder containing run_backend.py).
3. If retaining your existing virtual environment, run:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_backend.py --viewer --walk-mode scripted
```

For a fresh extraction, create the environment first:

```powershell
py -3.12 -m venv .venv
```

In a second terminal:

```powershell
cd mobile_app
flutter pub get
flutter run
```

Set Server connection to your laptop's current Wi-Fi address. Your last supplied
address was http://192.168.0.93:8000. The phone must be on the same Wi-Fi.

## What to expect

Press STAND once, let it complete, set the slider to 0.50 or 1.00, then press WALK.
The robot first bends its knees and transfers weight. The first foot lift begins
about 7–9 seconds after WALK. It then takes six slow steps and settles. At 0.50,
a complete run takes about 61 seconds of simulation time; at 1.00, about 52.
A busy laptop may run slower than simulation time. Do not press Stop while
waiting for the first foot lift unless you intend to cancel the walk.

A subsequent WALK continues from the current position. The existing STAND
command still resets the model to its original standing keyframe.

STOP during a walk changes status to STOPPING: the current phase finishes,
the foot lands, then the robot centers its weight. Allow up to approximately
15 seconds of simulation time for this controlled stop. A detected fall still
cancels immediately. This is a simulation stop, not a physical emergency stop.

The slider's 0–1 value now controls scripted cadence. It is NOT actual velocity
in m/s; 1.0 is the fastest tested conservative setting, not 1 m/s. Zero rejects a
new WALK. The /robot/speed endpoint can update cadence at the next step; the
mobile slider sends its selected value with the next WALK. RL mode remains an
optional external controller and retains its previous requested-speed semantics.

## Changes

- New motioncore/scripted_walk.py: separate IK working data, level foot targets,
  COM transfer, 3.5 cm swing clearance, 6 cm footstep spacing, support torque
  compensation and small ankle balance corrections.
- motioncore/motion.py: the existing SLOW_FORWARD_WALK_V1 state delegates to
  the improved gait. It no longer resets the robot to the origin for each WALK.
- config/motion_config.json: a new balanced_walk configuration. Legacy motion
  configurations remain available.
- backend/simulation_runtime.py: forwards pace, handles controlled stop, and
  limits viewer synchronization to 30 Hz while keeping physics steps unchanged.
- backend/robot_controller.py: reports STOPPING until settling finishes and
  forwards cadence updates.
- mobile_app/lib/controllers/dashboard_controller.dart: blocks new motion
  commands while STOPPING.
- mobile_app/lib/screens/home_page.dart: explains pace and stop behavior and
  removes misleading m/s units from the shared numerical setting.
- tests/test_balanced_walk.py: physics regressions for displacement, both foot
  lifts, repeated walks, stop phases, and pace handling.

## Measured validation

Headless MuJoCo 3.13.0, included G1 model and flat floor, normal gravity/contact
physics. No live base-position edits, applied support forces, or model changes.

| Pace | First walk forward | Second walk forward | Simulation time per walk |
|---|---:|---:|---:|
| 0.05 | 0.334 m | 0.370 m | 69.1 s |
| 0.50 | 0.334 m | 0.368 m | 61.0 s |
| 1.00 | 0.333 m | 0.365 m | 52.0 s |

Validation: 34 existing Python tests and 7 new walking regression cases passed.

All six recorded runs stayed upright. Maximum absolute roll was under 0.040 rad
and minimum pelvis height remained above 0.750 m. Both feet lifted over 3 cm.
The second walk's greater displacement reflects the staggered initial stance.
Raw metrics are in outputs/balanced_walk_validation.json.

Run the regression suite from the project folder:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

This is a slow flat-floor simulation gait, not a trained locomotion policy or
validated hardware controller. Rough terrain, disturbances, arbitrary starting
poses and real hardware are outside these tests. Flutter/Windows graphical
execution must still be checked on your laptop; Flutter was unavailable in the
build environment.

Implementation reference: MuJoCo Jacobian and dynamics API:
https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html
