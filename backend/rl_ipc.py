"""Atomic JSON IPC with Windows-compatible reader sharing and bounded retries.

Windows readers must grant FILE_SHARE_DELETE so os.replace can publish the next
snapshot while the previous snapshot is being read. The old handle still reads
its complete old snapshot; new readers receive the complete replacement.
"""
import errno
import json
import os
from pathlib import Path
import tempfile
import time


def sharing_error(exc):
    return isinstance(exc, OSError) and (
        getattr(exc, 'winerror', None) in (5, 32, 33)
        or exc.errno in (errno.EACCES, errno.EPERM, errno.EBUSY)
    )


def _retry(operation, timeout=.5):
    deadline = time.monotonic()+timeout
    delay = .002
    while True:
        try:
            return operation()
        except OSError as exc:
            remaining = deadline-time.monotonic()
            if not sharing_error(exc) or remaining <= 0:
                raise
            time.sleep(min(delay, remaining))
            delay = min(delay*2, .025)


def atomic_json(path, payload):
    path = Path(path)
    # Validate before touching the last valid snapshot.
    content = json.dumps(payload, allow_nan=False)
    fd, name = _retry(lambda: tempfile.mkstemp(prefix=path.name+'.', suffix='.tmp', dir=path.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(content)
        _retry(lambda: os.replace(temporary, path))
    finally:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            # A scanner can briefly hold a temp file too. Never mask the real
            # write error; runtime session folders can be removed after exit.
            pass


def _open_shared(path):
    if os.name != 'nt':
        return Path(path).open('r', encoding='utf-8')
    import ctypes
    from ctypes import wintypes
    import msvcrt
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    create = kernel.CreateFileW
    create.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                       ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    create.restype = wintypes.HANDLE
    close = kernel.CloseHandle
    close.argtypes = [wintypes.HANDLE]
    close.restype = wintypes.BOOL
    # GENERIC_READ; FILE_SHARE_READ | WRITE | DELETE; OPEN_EXISTING.
    raw = create(str(path), 0x80000000, 0x1 | 0x2 | 0x4, None, 3, 0x80, None)
    if raw == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        fd = msvcrt.open_osfhandle(raw, os.O_RDONLY | os.O_BINARY)
    except BaseException:
        close(raw)
        raise
    try:
        return os.fdopen(fd, 'r', encoding='utf-8')
    except BaseException:
        os.close(fd)
        raise


def read_json(path):
    def read():
        # Close the OS handle immediately after copying the bytes. Parsing does
        # not need to hold the file open.
        with _open_shared(path) as handle:
            text = handle.read()
        return json.loads(text)
    return _retry(read)


def publish_json(path, payload):
    """Telemetry only: skip a locked snapshot and retry at the next publish tick.

    Command writers continue using atomic_json, which raises if delivery fails.
    Heartbeat and stale-telemetry checks remain active during persistent failures.
    """
    try:
        atomic_json(path, payload)
        return True
    except OSError as exc:
        if not sharing_error(exc):
            raise
        return False
