from __future__ import annotations

import subprocess
from dataclasses import dataclass
from io import BytesIO
from typing import Iterable, TYPE_CHECKING, Optional

from loguru import logger
if TYPE_CHECKING:
    from PIL import Image

DEFAULT_ADB_BASE_PORT = 5555
DEFAULT_ADB_HOST = "127.0.0.1"


@dataclass(frozen=True)
class AGTDevice:
    """Agent device wrapper providing redroid management plus maatouch touch API.

    This keeps existing redroid functionality and delegates touch actions to
    `pymaatouch.MNTDevice` so callers can call `tap`, `swipe`, `pinch_zoom`, etc.
    """
    device_id: int
    adb_host: str = DEFAULT_ADB_HOST

    @property
    def adb_port(self) -> int:
        return DEFAULT_ADB_BASE_PORT + self.device_id

    @property
    def adb_target(self) -> str:
        return f"{self.adb_host}:{self.adb_port}"

    # Internal maatouch device, created after connect()
    _touch_device: Optional["MNTDevice"] = None

    def _run(self, cmd: list[str], check: bool = True) -> subprocess.CompletedProcess:
        logger.debug("Running command: {}", " ".join(cmd))
        return subprocess.run(cmd, check=check, text=True, capture_output=True)

    def _run_binary(self, cmd: list[str], check: bool = True) -> subprocess.CompletedProcess:
        logger.debug("Running command (binary): {}", " ".join(cmd))
        return subprocess.run(cmd, check=check, text=False, capture_output=True)

    def _adb(self, args: Iterable[str], check: bool = True) -> subprocess.CompletedProcess:
        cmd = ["adb", "-s", self.adb_target, *args]
        return self._run(cmd, check=check)

    def _adb_shell(self, shell_cmd: str, check: bool = True) -> subprocess.CompletedProcess:
        return self._adb(["shell", shell_cmd], check=check)

    def connect(self, retries: int = 3) -> None:
        self._run(["adb", "disconnect", self.adb_target], check=False)
        for _ in range(retries):
            res = self._run(["adb", "connect", self.adb_target], check=False)
            if "connected" in (res.stdout or "").lower():
                # After adb connect, create maatouch touch helper so callers can
                # use `tap`, `swipe`, etc., directly on this object.
                try:
                    from pymaatouch import MNTDevice as MNTDeviceImpl

                    # create non-frozen attribute via object.__setattr__
                    object.__setattr__(self, "_touch_device", MNTDeviceImpl(self.adb_target))
                except Exception:
                    # if maatouch not available or device unreachable, leave touch device None
                    object.__setattr__(self, "_touch_device", None)
                return
        raise RuntimeError(f"ADB connect failed: {self.adb_target}")

    # Key events
    def key_back(self) -> None:
        self._adb_shell("input keyevent 4")

    def key_home(self) -> None:
        self._adb_shell("input keyevent 3")

    def key_menu(self) -> None:
        self._adb_shell("input keyevent 82")

    def key_volume_up(self) -> None:
        self._adb_shell("input keyevent 24")

    def key_volume_down(self) -> None:
        self._adb_shell("input keyevent 25")

    # Touch delegation (powered by pymaatouch.MNTDevice)
    def _ensure_touch(self) -> None:
        if getattr(self, "_touch_device", None) is None:
            raise RuntimeError("touch device not initialized; call connect() first")

    def tap(self, points, pressure: int = 100, duration: int | None = None, no_up: bool | None = None):
        self._ensure_touch()
        return self._touch_device.tap(points, pressure=pressure, duration=duration, no_up=no_up)

    def swipe(self, points, pressure: int = 100, duration: int | None = None, no_down: bool | None = None, no_up: bool | None = None):
        self._ensure_touch()
        return self._touch_device.swipe(points, pressure=pressure, duration=duration, no_down=no_down, no_up=no_up)

    def ext_smooth_swipe(self, points, pressure: int = 100, duration: int | None = None, part: int | None = None, no_down: bool | None = None, no_up: bool | None = None):
        self._ensure_touch()
        return self._touch_device.ext_smooth_swipe(points, pressure=pressure, duration=duration, part=part, no_down=no_down, no_up=no_up)

    def pinch_zoom(self, points, scale, pressure: int = 100, duration: int = 500, steps: int = 10):
        self._ensure_touch()
        return self._touch_device.pinch_zoom(points, scale, pressure=pressure, duration=duration, steps=steps)

    def start(self):
        if getattr(self, "_touch_device", None) is not None:
            try:
                self._touch_device.start()
            except Exception:
                pass

    def stop(self):
        if getattr(self, "_touch_device", None) is not None:
            try:
                self._touch_device.stop()
            except Exception:
                pass

    def reset(self):
        if getattr(self, "_touch_device", None) is not None:
            try:
                self._touch_device.reset()
            except Exception:
                pass

    # Screenshot
    def screenshot(self) -> "Image.Image":
        cmd = ["adb", "-s", self.adb_target, "exec-out", "screencap", "-p"]
        res = self._run_binary(cmd, check=True)
        from PIL import Image as PILImage

        return PILImage.open(BytesIO(res.stdout))

    # Focus info
    def get_focus(self) -> dict[str, str]:
        res = self._adb_shell("dumpsys window | grep -E 'mCurrentFocus|mFocusedApp'", check=False)
        lines = [line.strip() for line in (res.stdout or "").splitlines() if line.strip()]
        return {
            "raw": "\n".join(lines),
            "mCurrentFocus": next((l for l in lines if "mCurrentFocus" in l), ""),
            "mFocusedApp": next((l for l in lines if "mFocusedApp" in l), ""),
        }

    # App management
    def _get_main_activity(self, package_name: str) -> str | None:
        """Find the main launcher activity for the given package."""
        # Method 1: Use cmd package resolve-activity (reliable on newer Android)
        res = self._adb_shell(f"cmd package resolve-activity --brief {package_name}", check=False)
        if res.returncode == 0 and res.stdout:
            lines = [l.strip() for l in res.stdout.splitlines() if l.strip()]
            if lines:
                # The format is usually:
                # priority=0 preferredOrder=0 match=0x108000 specificIndex=-1 isDefault=false
                # com.package/.Activity
                last_line = lines[-1]
                if "/" in last_line:
                    return last_line

        # Method 2: Fallback to dumpsys package (common)
        res = self._adb_shell(f"dumpsys package {package_name}", check=False)
        if res.stdout:
            import re

            # Look for the MAIN action and LAUNCHER category section
            output = res.stdout
            # Search for the block starting with android.intent.action.MAIN:
            # and then find the first activity that follows it before the next action
            main_block_match = re.search(
                r"android\.intent\.action\.MAIN:(.*?)(?:\w+\.intent\.action|$)",
                output,
                re.DOTALL,
            )
            if main_block_match:
                block = main_block_match.group(1)
                # In this block, find the activity that has the LAUNCHER category
                # Line format: <hash> <package>/<activity> filter <hash>
                if "android.intent.category.LAUNCHER" in block:
                    activity_match = re.search(r"(\S+/\S+)", block)
                    if activity_match:
                        return activity_match.group(1)

        return None

    def list_apps(
        self,
        include_system: bool = False,
        include_user: bool = True,
        include_label: bool = False,
    ) -> list[dict[str, str | None]]:
        packages: set[str] = set()
        if include_user:
            res = self._adb_shell("pm list packages -3", check=False)
            packages.update(_parse_packages(res.stdout))
        if include_system:
            res = self._adb_shell("pm list packages -s", check=False)
            packages.update(_parse_packages(res.stdout))

        apps: list[dict[str, str | None]] = []
        for pkg in sorted(packages):
            label = None
            if include_label:
                label = _get_app_label(self, pkg)
            apps.append({"package": pkg, "label": label})
        return apps

    def launch_app(self, package_name: str) -> None:
        # Try to find the main activity dynamically
        main_activity = self._get_main_activity(package_name)

        if main_activity:
            cmd = f"am start -n {main_activity}"
        else:
            # Fallback to the old behavior if we couldn't find the activity
            logger.warning(
                "could not determine main activity for %s, falling back to .MainActivity",
                package_name,
            )
            cmd = f"am start -a android.intent.action.MAIN -c android.intent.category.LAUNCHER -n {package_name}/.MainActivity"

        res = self._adb_shell(cmd, check=False)
        if res.returncode != 0:
            logger.warning("failed to start %s: %s %s", package_name, res.stdout, res.stderr)


def _parse_packages(output: str | None) -> list[str]:
    if not output:
        return []
    pkgs = []
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("package:"):
            pkgs.append(line.split(":", 1)[1])
    return pkgs


def _get_app_label(device: AGTDevice, package_name: str) -> str | None:
    res = device._adb_shell(f"dumpsys package {package_name} | grep -m1 application-label", check=False)
    if not res.stdout:
        return None
    line = res.stdout.strip()
    if ":" in line:
        return line.split(":", 1)[1].strip() or None
    return None


def get_device(device_id: int, adb_host: str = DEFAULT_ADB_HOST) -> AGTDevice:
    return AGTDevice(device_id=device_id, adb_host=adb_host)
