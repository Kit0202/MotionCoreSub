# Camera-guided selection from four cans

Start the pickup backend to display Cola, Tuna, 100PLUS and Pepsi **together on
one table**. Each is a separate free body with its own size, mass, collision
geometry and label. The app's dropdown selects the requested pickup type; it
never replaces, moves or resets any can.

## Install and run

Stop the old backend first, then from the project folder:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-vision.txt
.\RUN_PICKUP.bat
```

In another terminal:

```powershell
cd mobile_app
flutter pub get
flutter analyze
flutter test
flutter run
```

Use the app's Server connection setting, for example `http://192.168.0.93:8000`
while that is the laptop's current LAN address. Phone and laptop need a reachable
network. Only one backend should use port 8000. `RUN_RL.bat` retains the separate
walking/gesture scene.

## App workflow

1. All four cans are already on the table.
2. Choose **Can to pick up**, for example Pepsi.
3. **Find selected can** scans every visible tabletop object. The image marks
   recognised cans and highlights the selected match in green. The app lists
   all recognised types and the selected match's confidence and world position.
4. **Detect & pick up** always takes a fresh RGB-D scan, classifies the objects,
   finds exactly one match to the requested type, then plans from that object's
   depth-estimated position. Missing, uncertain or duplicate matches block motion.
5. Wait for **HOLDING**. The can remains physically held until **Place can**.
6. Place releases it at clearance height (about 13 cm above its starting center)
   so the unfolding fingers clear other cans. A calibrated small horizontal return motion
   compensates for the offset of the can inside the closed fingers. It settles
   onto the table under
   gravity; this is a short-drop release, not a precise contact placement. Choose another type and repeat without
   resetting the scene. A new scan accounts for the actual placed position.

The nearer hand is selected from the detected object's side of the table. The
left hand uses mirrored finger targets. The short tuna can uses a small wrist
roll to keep its contacts on the can sides. Vertical approach waypoints provide clearance between adjacent cans. After
release the arm withdraws behind the table to clear the camera view. The scene has been tested for this
spaced layout; it is not a general collision-avoidance planner.

**Stop** pauses the arm while retaining its grip target; physics and balance
continue. **Continue** resumes a paused pickup or placement. Selection is blocked
while moving, paused or holding. **Reset scene** explicitly restores the robot
and all four cans; it retains the requested type. It is the only normal app
operation that resets object poses. Reset after a failed grasp or lost balance.

## Perception and physics

A torso-mounted, downward-looking simulated RGB-D camera renders RGB and metric
depth. Calibrated depth points above the known 85 cm tabletop are clustered in
an XY occupancy grid. Each separate object gets a masked 64x64 RGB crop, classified
by the bundled CNN. Type matching happens after classification. The detector
receives images, camera calibration and the requested type; it never receives
object IDs, selected-object coordinates or simulator segmentation ground truth.

The arm's target position comes from camera depth, not a predefined slot. Tests
also swap Cola and Pepsi slots: the requested red can must be located and picked
up at its new position. Simulator object IDs are used separately to verify
physical contact/holding of the requested body, not as classifier inputs.

The robot uses the bundled articulated three-finger G1 model, joint torques and
a double-support standing controller. All four cans retain free joints. Grasping
uses opposing finger contact and friction, with no weld, mocap attachment, root
pinning, object teleport during pickup or applied support force. HOLDING requires
60 samples over three simulated seconds with opposing contact, more than 8 cm
of lift, and no table support. Untargeted cans must remain within 15 mm of their
settled positions during tested pickups and placement.

| Can | Diameter | Height | Simulated mass |
|---|---:|---:|---:|
| Cola | 65 mm | 115 mm | 352 g |
| Tuna | 82 mm | 58 mm | 182 g |
| 100PLUS | 63 mm | 125 mm | 342 g |
| Pepsi | 65 mm | 120 mm | 352 g |

These are stylised presets, not manufacturer-certified specifications or exact
branded assets. The CNN was trained on synthetic crops of these four designs;
`vision_assets/training.json` records synthetic validation. Colors and shapes
are its principal cues. There is no label OCR or real-can image training. Green
cans are the trained unknown examples; rejection of arbitrary unknown objects
is not guaranteed. This update does not connect a physical robot camera.

Known tabletop calibration, upright cans, a clear view and adequate spacing are
required. Touching/occluded objects may merge or fail localisation. Missing,
uncertain (<90% confidence), duplicate or out-of-reach target detections prevent
arm motion. Arbitrary clutter, obstacle avoidance, walking toward cans, carrying
while walking and real hardware are outside this demonstration's validation.

## API and validation

- `POST /robot/can` with `{"canType":"pepsi"}` changes only the target.
- `POST /robot/action` with `{"action":"SCAN"}` scans and identifies that target.
- `GRAB` scans afresh, then picks the matched object; `PLACE` returns it.
- `GET /robot/status` supplies `selectedCan`, `tableCans`, actions and `vision`.
  `vision.detections` contains all camera candidates; the top-level match is the
  selected target. `TARGET_NOT_FOUND` or `AMBIGUOUS` prevents pickup.
- `GET /robot/camera` returns the last annotated JPEG, or 404 before a scan.
  This is a snapshot, not live video. Target changes/reset invalidate it.

Camera rendering/inference runs on the simulation owner thread. Image requests
return cached bytes and never create rendering contexts.

```powershell
.\.venv\Scripts\python.exe -m pytest -q tests/test_multi_can_vision.py tests/test_can_selection.py tests/test_can_vision.py tests/test_pickup_controller.py tests/test_server.py tests/test_can_pickup.py
```

Headless Linux needs `MUJOCO_GL=egl`. Flutter runtime/layout checks must run
locally; parsing Dart source alone does not verify the phone app.

The standalone `python scripts/grab_can.py --headless --can tuna` retains its
single-can, preset-position physics evaluation. The four-can scene and camera
selection workflow are provided by the pickup backend/mobile app.
