"""Cooperative per-device locks. Never kill another owner's process."""
import fcntl
import hashlib
import os
from pathlib import Path
import tempfile


class DeviceLock:
    def __init__(self, identity):
        digest = hashlib.sha256(str(identity).encode()).hexdigest()[:24]
        self.path = Path(tempfile.gettempdir()) / ("spr_device_" + digest + ".lock")
        self.fd = None

    def acquire(self):
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, BlockingIOError):
            os.close(fd)
            raise RuntimeError(f"device already owned; lock={self.path}") from None
        self.fd = fd
        os.ftruncate(fd, 0)
        os.write(fd, str(os.getpid()).encode())
        return self

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        # Do not unlink: waiters must lock the same inode.


def verify_usb_camera(serial, sysfs=Path("/sys/bus/usb/devices")):
    for path in sysfs.iterdir():
        try:
            if (path / "serial").read_text().strip() != serial:
                continue
            speed = float((path / "speed").read_text().strip())
            if speed < 5000:
                raise RuntimeError(f"camera USB speed is {speed} Mbit/s; USB3 required")
            return {"serial_no": serial, "usb_path": str(path.resolve()), "speed_mbps": speed}
        except (FileNotFoundError, PermissionError):
            continue
    raise RuntimeError(f"RealSense serial {serial} not connected")


def verify_serial_free(device):
    target = str(Path(device).resolve(strict=True))
    owners = []
    for fd in Path("/proc").glob("[0-9]*/fd/*"):
        try:
            if str(fd.resolve()) == target:
                owners.append(fd.parts[2])
        except (OSError, RuntimeError):
            pass
    if owners:
        raise RuntimeError(f"serial device is already open by PID(s): {sorted(set(owners))}")
    return target
