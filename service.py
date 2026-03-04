from __future__ import annotations

import subprocess
from dataclasses import dataclass
from io import BytesIO
from typing import Iterable, TYPE_CHECKING, Optional, Any

import ast
import os
import re
import shutil
import time

from loguru import logger
from .constants import (
    DEFAULT_ADB_BASE_PORT,
    DEFAULT_ADB_HOST,
    DEFAULT_TOUCH_MARGIN_MS,
    RECORD_BEFORE,
    RECORD_TIME,
    SYSTEM_PROMPT,
)
from .tools import get_tool_entry
from .tools.prototype import CallContext, call as call_tool
from .trajectory import ToolCallTrajectory

if TYPE_CHECKING:
    from PIL import Image



@dataclass(frozen=True)
class AGTDevice:
    """Agent device wrapper providing redroid management plus maatouch touch API.

    This keeps existing redroid functionality and delegates touch actions to
    `pymaatouch.MNTDevice` so callers can call `tap`, `swipe`, `pinch_zoom`, etc.
    """
    device_id: int
    adb_host: str = DEFAULT_ADB_HOST
    width: Optional[int] = None
    height: Optional[int] = None

    def __post_init__(self) -> None:
        try:
            import yaml

            base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_path = os.path.join(base_path, "device_config.yaml")
            if not os.path.exists(config_path):
                config_path = os.path.join(os.getcwd(), "device_config.yaml")

            w = self.width
            h = self.height
            if (w is None or h is None) and os.path.exists(config_path):
                with open(config_path, "r") as f:
                    cfg = yaml.safe_load(f)
                    if cfg:
                        for item in cfg:
                            dev_cfg = item.get("device", {})
                            if dev_cfg.get("id") == self.device_id:
                                res = dev_cfg.get("screen_resolution")
                                if isinstance(res, str) and "x" in res:
                                    try:
                                        w_s, h_s = res.split("x")
                                        w = int(w_s)
                                        h = int(h_s)
                                    except Exception:
                                        pass
                                break

            if w is None:
                w = 1080
            if h is None:
                h = 1920

            object.__setattr__(self, "width", w)
            object.__setattr__(self, "height", h)
        except Exception as e:
            logger.warning(f"AGTDevice __post_init__ load config failed: {e}")
            if self.width is None:
                object.__setattr__(self, "width", 1080)
            if self.height is None:
                object.__setattr__(self, "height", 1920)

    @property
    def adb_port(self) -> int:
        return DEFAULT_ADB_BASE_PORT + self.device_id

    @property
    def adb_target(self) -> str:
        return f"{self.adb_host}:{self.adb_port}"

    _touch_device: Optional["AGTDevice"] = None
    _is_holding: bool = False
    _hold_locked: bool = False
    _trajectory_recorder: Optional[ToolCallTrajectory] = None

    def is_holding(self) -> bool:
        return getattr(self, "_is_holding", False)

    def is_hold_locked(self) -> bool:
        return getattr(self, "_hold_locked", False)

    def get_system_prompt(self) -> str:
        prompt = SYSTEM_PROMPT + "\n当前touch.continus.down()激活，你正在持续按压屏幕，只能调用 touch.continus.move_to() 和 wait()，touch.continus.up()松开"
        if self.is_hold_locked():
            prompt += "\n\n正在持续按压屏幕，只能调用 touch.continus.move_to() 和 wait()，touch.continus.up()松开"
        return prompt

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
                try:
                    from pymaatouch import MNTDevice as MNTDeviceImpl

                    object.__setattr__(self, "_touch_device", MNTDeviceImpl(self.adb_target))
                except Exception:
                    object.__setattr__(self, "_touch_device", None)
                return
        raise RuntimeError(f"ADB connect failed: {self.adb_target}")

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

    def _ensure_touch(self) -> None:
        if getattr(self, "_touch_device", None) is None:
            try:
                self.connect()
            except Exception:
                pass
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

    def up(self, contact_id: int = 0):
        self._ensure_touch()
        return self._touch_device.up(contact_id)

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

    def screenshot(self) -> "Image.Image":
        cmd = ["adb", "-s", self.adb_target, "exec-out", "screencap", "-p"]
        res = self._run_binary(cmd, check=True)
        from PIL import Image as PILImage

        return PILImage.open(BytesIO(res.stdout))

    def get_focus(self) -> dict[str, str]:
        res = self._adb_shell("dumpsys window | grep -E 'mCurrentFocus|mFocusedApp'", check=False)
        lines = [line.strip() for line in (res.stdout or "").splitlines() if line.strip()]
        return {
            "raw": "\n".join(lines),
            "mCurrentFocus": next((l for l in lines if "mCurrentFocus" in l), ""),
            "mFocusedApp": next((l for l in lines if "mFocusedApp" in l), ""),
        }

    def _get_main_activity(self, package_name: str) -> str | None:
        res = self._adb_shell(f"cmd package resolve-activity --brief {package_name}", check=False)
        if res.returncode == 0 and res.stdout:
            lines = [l.strip() for l in res.stdout.splitlines() if l.strip()]
            if lines:
                last_line = lines[-1]
                if "/" in last_line:
                    return last_line

        res = self._adb_shell(f"dumpsys package {package_name}", check=False)
        if res.stdout:
            main_block_match = re.search(
                r"android\.intent\.action\.MAIN:(.*?)(?:\w+\.intent\.action|$)",
                res.stdout,
                re.DOTALL,
            )
            if main_block_match:
                block = main_block_match.group(1)
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
        main_activity = self._get_main_activity(package_name)

        if main_activity:
            cmd = f"am start -n {main_activity}"
        else:
            logger.warning(
                "could not determine main activity for %s, falling back to .MainActivity",
                package_name,
            )
            cmd = f"am start -a android.intent.action.MAIN -c android.intent.category.LAUNCHER -n {package_name}/.MainActivity"

        res = self._adb_shell(cmd, check=False)
        if res.returncode != 0:
            logger.warning("failed to start %s: %s %s", package_name, res.stdout, res.stderr)

    def init_trajectory_recording(self, output_dir: str | None = None, session_name: str | None = None) -> str:
        base_dir = output_dir or os.path.join(os.getcwd(), "logs")
        recorder = ToolCallTrajectory(root_dir=base_dir, session_name=session_name)
        object.__setattr__(self, "_trajectory_recorder", recorder)
        logger.info(f"Trajectory recording started: {recorder.session_dir}")
        return recorder.session_dir

    def finish_trajectory_recording(self) -> str | None:
        recorder = getattr(self, "_trajectory_recorder", None)
        if recorder is None:
            return None
        json_path = recorder.finish()
        object.__setattr__(self, "_trajectory_recorder", None)
        logger.info(f"Trajectory recording finished: {json_path}")
        return json_path

    def replay_trajectory(self, trajectory_path_or_dir: str, respect_interval: bool = True) -> list[tuple[str, str | None]]:
        batches = ToolCallTrajectory.load_batches(trajectory_path_or_dir)
        replay_results: list[tuple[str, str | None]] = []
        for batch in batches:
            call_string = str(batch.get("call_string", ""))
            get_video = bool(batch.get("get_video", False))
            replay_results.append(self.agent_call(call_string, get_video=get_video))
            if respect_interval:
                interval = batch.get("interval_to_next_batch_sec")
                if isinstance(interval, (int, float)) and interval > 0:
                    time.sleep(interval)
        return replay_results

    def _record_trajectory_batch(
        self,
        *,
        started_at: float,
        ended_at: float,
        call_string: str,
        get_video: bool,
        tool_returns: list[dict],
        status_type: str,
        status_src_path: str | None,
        evidence_paths: list[str],
    ) -> None:
        recorder = getattr(self, "_trajectory_recorder", None)
        if recorder is None:
            return

        status_image = None
        if not status_src_path:
            try:
                status_image = self.screenshot().convert("RGB")
                status_type = "image"
            except Exception as e:
                logger.warning(f"Failed to capture status screenshot for trajectory: {e}")
                status_type = "none"

        recorder.add_batch(
            started_at=started_at,
            ended_at=ended_at,
            call_string=call_string,
            get_video=get_video,
            tool_returns=tool_returns,
            status_type=status_type,
            status_src_path=status_src_path,
            status_image=status_image,
            evidence_src_paths=evidence_paths,
        )

    def agent_call(
        self,
        call_string: str,
        get_video: bool = False,
        return_details: bool = False,
    ) -> tuple[str, str | None] | tuple[str, str | None, dict[str, Any]]:
        logger.info(f"agent_call: {call_string}")
        started_at = time.time()

        path_to_video = None
        screen_record_process = None
        remote_video_path = f"/sdcard/record_{int(time.time())}.mp4"
        errors: list[str] = []
        tool_returns: list[dict] = []
        evidence_paths: list[str] = []
        final_status = "success"

        if get_video:
            record_cmd = [
                "adb", "-s", self.adb_target, "shell",
                "screenrecord", f"--time-limit={int(RECORD_BEFORE + RECORD_TIME)}", remote_video_path,
            ]
            logger.debug(f"Starting screen record: {' '.join(record_cmd)}")
            screen_record_process = subprocess.Popen(record_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(RECORD_BEFORE)

        json_content = call_string
        code_block_match = re.search(r"```agent_call\s*(.*?)\s*```", call_string, re.DOTALL)
        if code_block_match:
            json_content = code_block_match.group(1)
        else:
            json_list_match = re.search(r"(\[.*\])", call_string, re.DOTALL)
            if json_list_match:
                json_content = json_list_match.group(1)

        json_content = json_content.replace("\t", "    ")

        def _escape_text_input_quotes(raw: str) -> str:
            pattern = r"text\.input\(\"([^\"]*?)\"\)"

            def _repl(m: re.Match[str]) -> str:
                return f"text.input(\\\"{m.group(1)}\\\")"

            return re.sub(pattern, _repl, raw)

        json_content = _escape_text_input_quotes(json_content)

        try:
            import yaml

            actions = yaml.safe_load(json_content)
        except Exception as e:
            logger.error(f"Parse error: {e}")
            logger.error(f"Failed content: {json_content!r}")
            ended_at = time.time()
            self._record_trajectory_batch(
                started_at=started_at,
                ended_at=ended_at,
                call_string=call_string,
                get_video=get_video,
                tool_returns=[{"error": f"parse error: {e}"}],
                status_type="none",
                status_src_path=None,
                evidence_paths=[],
            )
            details = {
                "started_at": started_at,
                "ended_at": ended_at,
                "duration_sec": round(max(0.0, ended_at - started_at), 3),
                "tool_returns": [{"error": f"parse error: {e}"}],
                "status_type": "none",
                "status_src_path": None,
                "evidence_paths": [],
            }
            if return_details:
                return "parse error", None, details
            return "parse error", None

        if not isinstance(actions, list):
            if isinstance(actions, dict):
                actions = [actions]
            else:
                logger.error(f"JSON is not a list or dict: {type(actions)}")
                ended_at = time.time()
                self._record_trajectory_batch(
                    started_at=started_at,
                    ended_at=ended_at,
                    call_string=call_string,
                    get_video=get_video,
                    tool_returns=[{"error": "json formatting error"}],
                    status_type="none",
                    status_src_path=None,
                    evidence_paths=[],
                )
                details = {
                    "started_at": started_at,
                    "ended_at": ended_at,
                    "duration_sec": round(max(0.0, ended_at - started_at), 3),
                    "tool_returns": [{"error": "json formatting error"}],
                    "status_type": "none",
                    "status_src_path": None,
                    "evidence_paths": [],
                }
                if return_details:
                    return "json formatting error", None, details
                return "json formatting error", None

        touch_consumers = {
            "touch.tap", "touch.long_tap", "touch.swipe", "touch.zoom",
            "touch.continus.down", "touch.continus.move_to",
        }
        implicit_tap_exempt = {"cature_evidence", "capture_evidence"}

        for action in actions:
            label = action.get("label", "")
            bbox = action.get("bbox_2d", None)

            if "#" in label:
                label = label.split("#", 1)[0]
            label = label.strip()

            func_match = re.match(r"^([\w\.]+)\((.*)\)$", label)
            if not func_match:
                logger.warning(f"Skipping invalid label: {label}")
                continue

            func_name, args_str = func_match.groups()
            pos_args = ()
            kw_args = {}
            if args_str.strip():
                try:
                    tree = ast.parse(f"f({args_str})", mode="eval")
                    call_node = tree.body

                    def _safe_eval(node):
                        try:
                            return ast.literal_eval(node)
                        except ValueError:
                            if isinstance(node, ast.Name):
                                return node.id
                            raise

                    pos_args = tuple(_safe_eval(a) for a in call_node.args)
                    kw_args = {k.arg: _safe_eval(k.value) for k in call_node.keywords}
                except Exception:
                    logger.warning(f"Failed to parse args with ast: {args_str}, fallback to literal_eval")
                    try:
                        pos_args = ast.literal_eval(f"({args_str},)")
                    except Exception:
                        if func_name == "text.input":
                            pos_args = (args_str,)
                        else:
                            logger.warning(f"Failed to parse args completely: {args_str}")
                            continue

            logger.debug(f"Executing {func_name} with pos_args={pos_args} kw_args={kw_args} bbox={bbox}")

            if self.is_hold_locked():
                allowed_funcs = ["touch.continus.move_to", "touch.continus.up", "wait"]
                if func_name not in allowed_funcs:
                    raise RuntimeError(
                        f"Constraint Violation: Device is in hold state. Only {allowed_funcs} are allowed. Got {func_name}"
                    )

            if getattr(self, "_last_touch_pos", None) and not func_name.startswith("touch.continus") and func_name != "wait":
                self.up()
                object.__setattr__(self, "_last_touch_pos", None)
                object.__setattr__(self, "_is_holding", False)
                object.__setattr__(self, "_hold_locked", False)

            did_implicit_tap = False
            if bbox and func_name not in touch_consumers and func_name not in implicit_tap_exempt:
                cx = (bbox[0] + bbox[2]) / 2
                cy = (bbox[1] + bbox[3]) / 2
                px = int(cx * self.width / 1000)
                py = int(cy * self.height / 1000)
                self.tap([(px, py)])
                time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
                did_implicit_tap = True

            entry = get_tool_entry(func_name)
            if entry is None:
                logger.warning(f"Unknown function: {func_name}")
                tool_returns.append({
                    "func_name": func_name,
                    "ok": False,
                    "error": "unknown function",
                    "action": action,
                })
                continue

            ctx = CallContext(
                device=self,
                action=action,
                func_name=func_name,
                bbox=bbox,
                args=pos_args,
                kw_args=kw_args,
                did_implicit_tap=did_implicit_tap,
            )

            result = call_tool(ctx, entry.handler, spec=entry.spec)
            tool_record = {
                "func_name": func_name,
                "ok": result.ok,
                "action": action,
                "did_implicit_tap": did_implicit_tap,
            }
            if result.value is not None:
                tool_record["value"] = result.value
            if result.error_message:
                tool_record["error"] = result.error_message
            tool_returns.append(tool_record)

            if result.ok and isinstance(result.value, str):
                if func_name in {"capture_evidence", "cature_evidence"}:
                    evidence_paths.append(result.value)

            if not result.ok:
                if result.exception and "Constraint Violation" in str(result.exception):
                    raise result.exception
                msg = result.error_message or f"Error executing action {action}"
                logger.error(msg)
                errors.append(msg)
                continue

            if result.done:
                msg = str(result.value) if result.value is not None else "task completed"
                logger.info(f"Task Done: {msg}")
                final_status = f"DONE: {msg}"
                break
        
        # End of actions check: Consistency check
        if self.is_holding() and not self.is_hold_locked():
             # Violates "Close in one round" rule
             self.up()
             object.__setattr__(self, "_last_touch_pos", None)
             object.__setattr__(self, "_is_holding", False)
             object.__setattr__(self, "_hold_locked", False)
             raise RuntimeError("Constraint Violation: touch.continus.down() used without hold=True must be closed by touch.continus.up() within the same agent_call.")

        if screen_record_process:
            # Wait for recording to finish
            try:
                # Wait for the recording time plus a small buffer
                timeout = RECORD_BEFORE + RECORD_TIME + 2
                screen_record_process.wait(timeout=timeout)
                
                # Pull video from device
                local_video_dir = os.path.join(os.getcwd(), "videos")
                os.makedirs(local_video_dir, exist_ok=True)
                local_video_path = os.path.join(local_video_dir, os.path.basename(remote_video_path))
                
                pull_cmd = ["adb", "-s", self.adb_target, "pull", remote_video_path, local_video_path]
                pull_res = self._run(pull_cmd, check=False)
                
                if pull_res.returncode == 0:
                    path_to_video = local_video_path
                    logger.info(f"Screen recording saved to {path_to_video}")
                else:
                    logger.error(f"Failed to pull screen recording: {pull_res.stderr}")

                # Clean up remote file
                self._adb_shell(f"rm {remote_video_path}", check=False)

            except subprocess.TimeoutExpired:
                logger.warning("Screen record process timed out.")
                screen_record_process.kill()
            except Exception as e:
                logger.error(f"Error during video processing: {e}")

        ended_at = time.time()
        status_type = "video" if path_to_video else "image"
        self._record_trajectory_batch(
            started_at=started_at,
            ended_at=ended_at,
            call_string=call_string,
            get_video=get_video,
            tool_returns=tool_returns,
            status_type=status_type,
            status_src_path=path_to_video,
            evidence_paths=evidence_paths,
        )

        status_value = f"error: {' | '.join(errors)}" if errors else final_status
        details = {
            "started_at": started_at,
            "ended_at": ended_at,
            "duration_sec": round(max(0.0, ended_at - started_at), 3),
            "tool_returns": tool_returns,
            "status_type": status_type,
            "status_src_path": path_to_video,
            "evidence_paths": evidence_paths,
        }
        if return_details:
            return status_value, path_to_video, details
        return status_value, path_to_video


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
