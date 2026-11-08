# MotionCore Submission / Demo Checklist

## Before submission

1. Keep `MotionCore_FYP_Report_Chapters_1_to_4.docx` and the revised proposal with the project submission files.
2. Do not submit `.venv`, `.git`, `.pytest_cache`, `__pycache__`, or Android `.gradle` cache if the portal has a size limit.
3. Run `python -m pytest` after installing `requirements.txt`.
4. From `mobile_app`, run `flutter pub get`, `flutter analyze`, and `flutter test` on your development laptop.
5. Do not claim real Unitree G1 hardware testing. `UnitreeSDK2Interface` is still a future integration layer.

## Fast demo tonight

```powershell
cd MotionCore
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python run_backend.py --viewer --walk-mode scripted
```

Open `http://127.0.0.1:8000/docs` in the laptop browser to test the API quickly.

For the phone app, run `ipconfig`, copy the laptop Wi-Fi IPv4 address, then enter `http://<IPv4>:8000` in the app's **Server connection** dialog. Phone and laptop must be on the same Wi-Fi.

Recommended demonstration order: **STAND -> WAVE -> SALUTE -> WALK -> STOP**. If fall detection is triggered, press **STOP**, then **STAND** to reset the simulation.

## What was completed in the submission-ready update

- WAVE, SALUTE and STAND now execute actual MotionCore MuJoCo states from the FastAPI/mobile path.
- WALK works without the external RL repository by using the included scripted MuJoCo forward-walk state; external RL remains optional.
- The MuJoCo simulator runs in a backend worker thread and exposes live telemetry.
- STOP cancels active MotionCore motion and terminates the optional RL process.
- A fall blocks new WALK/WAVE/SALUTE commands until the simulation is reset with STAND.
- Hard-coded `D:\...` backend paths were removed and replaced with project-relative paths/environment variables.
- External telemetry JSON path is configurable and stale samples are rejected.
- API actions and speed values are validated.
- Android local HTTP access is enabled for same-Wi-Fi FastAPI demonstrations.
- Empty pose and trajectory tests were completed, with additional backend controller tests.
