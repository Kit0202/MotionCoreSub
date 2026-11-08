"""Launch the bundled Unitree RL worker and exchange acknowledged commands."""
from __future__ import annotations
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
from backend.rl_ipc import atomic_json, read_json, sharing_error

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RLLocomotion:
    def __init__(self, *, headless=None, runtime_dir=None):
        self.headless = (os.getenv('MOTIONCORE_RL_HEADLESS', '0') == '1') if headless is None else headless
        self.runtime_dir = Path(runtime_dir or PROJECT_ROOT/'outputs'/'rl_runtime')
        self.process = None
        self._lock = threading.RLock()
        self._monitor_stop = threading.Event()
        self._thread = None
        self._log = None
        self.session_dir = None
        self.token = None
        self.seq = 0
        self.speed = 0.
        self.stopping = False
        self.last_error = None
        self._sample = None
        self._started = 0.

    def readiness(self):
        missing = []
        for package in ('torch', 'mujoco', 'yaml'):
            if importlib.util.find_spec(package) is None:
                missing.append(f'Python package: {package}')
        for path in ('backend/rl_worker.py', 'rl_assets/motion.pt', 'rl_assets/g1_motioncore.yaml',
                     'rl_assets/g1/scene.xml', 'rl_assets/g1/g1_12dof.xml'):
            if not (PROJECT_ROOT/path).is_file():
                missing.append(path)
        return dict(ready=not missing, missing=missing)

    def is_running(self):
        with self._lock:
            return self.process is not None and self.process.poll() is None

    def _write_command(self):
        atomic_json(self.session_dir/'command.json', dict(session=self.token, seq=self.seq,
                    speed=self.speed, stop=self.stopping, heartbeat=time.time()))

    def _read_sample(self):
        if self.session_dir is None:
            return None
        try:
            sample = read_json(self.session_dir/'telemetry.json')
            if sample.get('session') == self.token:
                self._sample = sample
        except (OSError, ValueError):
            pass
        return self._sample

    def start_walk(self, speed):
        with self._lock:
            if self.is_running():
                return dict(success=False, message='RL simulation is already running')
            ready = self.readiness()
            if not ready['ready']:
                return dict(success=False, message='Install requirements-rl.txt first', missing=ready['missing'])
            if not math.isfinite(speed) or not 0 < speed <= 1:
                return dict(success=False, message='Choose an RL speed above zero and at most 1 m/s')
            self.token = uuid.uuid4().hex
            self.session_dir = self.runtime_dir/self.token
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.speed, self.seq, self.stopping = float(speed), 1, False
            self._sample, self.last_error = None, None
            self._started = time.time()
            try:
                self._write_command()
                self._log = (self.session_dir/'worker.log').open('w', encoding='utf-8')
                command = [sys.executable, '-u', str(PROJECT_ROOT/'backend'/'rl_worker.py'),
                           '--session-dir', str(self.session_dir)]
                if self.headless:
                    command.append('--headless')
                self.process = subprocess.Popen(command, cwd=PROJECT_ROOT,
                                                stdout=self._log, stderr=subprocess.STDOUT)
            except OSError as exc:
                self.last_error = str(exc)
                if self._log:
                    self._log.close()
                return dict(success=False, message=f'Cannot launch RL: {exc}')
            if self._thread is None or not self._thread.is_alive():
                self._monitor_stop.clear()
                self._thread = threading.Thread(target=self._monitor, daemon=True)
                self._thread.start()
            return dict(success=True, status='STARTING', speed=speed, applied=False,
                        message='Loading RL policy; the walking window will open shortly')

    def _monitor(self):
        while not self._monitor_stop.wait(.25):
            with self._lock:
                if not self.is_running():
                    if self._log and not self._log.closed:
                        self._log.close()
                    continue
                try:
                    try:
                        self._write_command()
                    except OSError as exc:
                        if not sharing_error(exc):
                            raise
                        # The next heartbeat retries. A sustained outage is
                        # handled by the worker's existing five-second timeout.
                        pass
                    sample = self._read_sample()
                    age = time.time()-float(sample.get('timestamp', self._started)) if sample else time.time()-self._started
                    timeout = 60 if not sample or sample.get('status') == 'STARTING' else 10
                    if age > timeout:
                        self.last_error = 'RL worker stopped responding; simulation terminated'
                        self.process.terminate()
                except (OSError, ValueError) as exc:
                    self.last_error = f'RL communication failed: {exc}'
                    self.process.terminate()

    def change_speed(self, speed):
        with self._lock:
            if not math.isfinite(speed) or not 0 <= speed <= 1:
                return dict(success=False, message='Invalid RL speed')
            if not self.is_running() or self.stopping:
                return dict(success=False, message='RL is not accepting speed changes')
            if speed == 0:
                return self.stop_walk()
            self.speed, self.seq = float(speed), self.seq+1
            try:
                self._write_command()
            except OSError as exc:
                self.last_error = str(exc)
                return dict(success=False, message=f'Cannot send speed: {exc}')
            return dict(success=True, speed=speed, applied=False, commandSeq=self.seq,
                        message='Speed sent; telemetry confirms the applied command as it ramps')

    def stop_walk(self):
        with self._lock:
            if not self.is_running():
                return dict(success=True, stopping=False, message='RL simulation is stopped')
            self.speed, self.stopping, self.seq = 0., True, self.seq+1
            try:
                self._write_command()
            except OSError as exc:
                self.process.terminate()
                self.last_error = f'Controlled stop could not be sent; process termination requested: {exc}'
                return dict(success=False, stopping=True, message=self.last_error)
            return dict(success=True, stopping=True, message='RL is slowing to zero, then closing its simulation')

    def status(self):
        with self._lock:
            running = self.is_running()
            sample = self._read_sample()
            state = (sample or {}).get('status', 'STARTING' if running else 'IDLE')
            message = (sample or {}).get('message', '')
            if running and self.stopping and state not in ('ERROR', 'FALLEN', 'STOPPED'):
                state = 'STOPPING'
            if running and state == 'STOPPED':
                state, message = 'STOPPING', 'RL simulation is closing'
            if not running and self.process is not None and state not in ('STOPPED', 'FALLEN', 'ERROR'):
                state = 'ERROR'
                message = self.last_error or f'RL process exited ({self.process.returncode}); see {self.session_dir / "worker.log"}'
            if self.last_error:
                state, message = 'ERROR', self.last_error
            return {**self.readiness(), 'running': running, 'status': state,
                    'message': message, 'lastError': self.last_error,
                    'speed': self.speed, 'commandSeq': self.seq,
                    'appliedSeq': (sample or {}).get('commandSeq', 0),
                    'pid': self.process.pid if running else None}

    def get_telemetry(self):
        with self._lock:
            sample = self._read_sample()
            if sample is None:
                return dict(available=False, source='unitree_rl', height=0., roll=0., pitch=0.,
                            leftFoot=False, rightFoot=False, fallen=False, error='RL telemetry is loading')
            result = dict(sample)
            result['ageSeconds'] = max(0., time.time()-sample['timestamp'])
            if result['ageSeconds'] > 3:
                result['available'] = False
                result['error'] = 'RL telemetry is stale'
            return result

    def shutdown(self):
        self.stop_walk()
        process = self.process
        if process is not None:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        self._monitor_stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if self._log:
            self._log.close()
