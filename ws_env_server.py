"""
Unified WebSocket server for redroid environment control.

Design:
- A single endpoint serves multiple redroid devices.
- Each request must include `device_id`; server routes to that device runtime.
- Device id is validated against the provided device config yaml.
- Intended to be started with sudo/root (docker/adb operations).

Protocol (single JSON request -> single JSON response):
- Request common fields: `request_id`, `op`, `device_id`, optional `worker_idx`.
- Ops:
    - `reset`: initialize episode with task and return first observation.
    - `step`: execute `action_text` and return transition.
    - `close`: finish current trajectory recording.
    - `ping`: health check.
    - `monitor_state`: return monitor snapshot for UI sync.


sudo -v && mkdir -p /home/zch/Documents/AgentEnv/logs && nohup sudo -E /home/zch/Documents/AgentEnv/start_ws_env_server.sh > /home/zch/Documents/AgentEnv/logs/ws_env_server.log 2>&1
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import importlib
import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from io import BytesIO
from typing import Any, Dict

from loguru import logger

from redroid_keeper.constants import SYSTEM_PROMPT
from redroid_keeper.keeper import create_device, load_config, start_device
from redroid_keeper.ws_monitor import run_monitor_ui
from redroid_keeper.service import AGTDevice


@dataclass
class DeviceRuntime:
    device: AGTDevice
    lock: threading.Lock = field(default_factory=threading.Lock)
    task: Dict[str, Any] = field(default_factory=dict)
    step_count: int = 0
    max_steps: int = 30
    trajectory_path: str | None = None


def _encode_image_to_b64(image) -> str:
    buf = BytesIO()
    image.convert("RGB").save(buf, format="JPEG", quality=75)
    return base64.b64encode(buf.getvalue()).decode("utf-8")


class RedroidWSService:
    """Stateful multi-device redroid runtime manager for WS requests."""

    def __init__(
        self,
        adb_host: str = "127.0.0.1",
        trajectory_root: str = "logs/redroid_rollout",
        config_path: str | None = None,
    ):
        self.adb_host = adb_host
        self.trajectory_root = trajectory_root
        self.config_path = config_path
        self._runtimes: dict[int, DeviceRuntime] = {}
        self._global_lock = threading.Lock()
        self._allowed_device_ids: set[int] | None = None
        self._device_cfg_by_id: dict[int, Any] = {}

        self._monitor_lock = threading.Lock()
        self._monitor_events: deque[dict[str, Any]] = deque(maxlen=300)
        self._monitor_state: dict[int, dict[str, Any]] = {}

        if self.config_path:
            devices = load_config(self.config_path)
            self._allowed_device_ids = {int(d.device_id) for d in devices}
            self._device_cfg_by_id = {int(d.device_id): d for d in devices}
            logger.info(f"Loaded device config from {self.config_path}, device_ids={sorted(self._allowed_device_ids)}")

    @staticmethod
    def _docker_container_exists(container_name: str) -> bool:
        res = subprocess.run(
            ["docker", "ps", "-a", "--filter", f"name=^{container_name}$", "-q"],
            capture_output=True,
            text=True,
            check=False,
        )
        return bool((res.stdout or "").strip())

    @staticmethod
    def _docker_container_running(container_name: str) -> bool:
        res = subprocess.run(
            ["docker", "ps", "--filter", f"name=^{container_name}$", "-q"],
            capture_output=True,
            text=True,
            check=False,
        )
        return bool((res.stdout or "").strip())

    def _ensure_device_awake(self, device_id: int) -> None:
        cfg = self._device_cfg_by_id.get(device_id)
        if cfg is None:
            return

        exists = self._docker_container_exists(cfg.container_name)
        running = self._docker_container_running(cfg.container_name) if exists else False

        if not exists:
            logger.info(f"device_id={device_id} not found as container, creating")
            create_device(cfg)
            return

        if not running:
            logger.info(f"device_id={device_id} container exists but stopped, starting")
            start_device(cfg)

    def _record_event(self, device_id: int, op: str, payload: Dict[str, Any], response: Dict[str, Any]) -> None:
        with self._monitor_lock:
            info = response.get("info", {}) if isinstance(response, dict) else {}
            tool_returns = info.get("tool_returns", []) if isinstance(info, dict) else []
            tool_actions: list[str] = []
            if isinstance(tool_returns, list):
                for tr in tool_returns[:20]:
                    if not isinstance(tr, dict):
                        continue
                    action = tr.get("action", {})
                    if isinstance(action, dict):
                        label = str(action.get("label", "")).strip()
                        if label:
                            tool_actions.append(label)

            event = {
                "ts": time.time(),
                "device_id": device_id,
                "op": op,
                "request_id": payload.get("request_id"),
                "ok": bool(response.get("ok", False)),
                "error": response.get("error", ""),
                "action_text": payload.get("action_text", ""),
                "executor_status": info.get("executor_status", ""),
                "tool_actions": tool_actions,
                "reward": float(response.get("reward", 0.0) or 0.0),
                "done": bool(response.get("done", False)),
            }
            self._monitor_events.append(event)

            st = self._monitor_state.get(device_id, {})
            st.update(
                {
                    "updated_at": event["ts"],
                    "device_id": device_id,
                    "last_op": op,
                    "last_request": {
                        "request_id": payload.get("request_id"),
                        "action_text": payload.get("action_text", ""),
                        "task": payload.get("task", {}),
                    },
                    "last_response": {
                        "ok": response.get("ok", False),
                        "error": response.get("error", ""),
                        "reward": response.get("reward", 0.0),
                        "done": response.get("done", False),
                        "observation_text": response.get("observation_text", ""),
                        "info": response.get("info", {}),
                        "image_b64": response.get("image_b64", ""),
                    },
                }
            )
            self._monitor_state[device_id] = st

    def monitor_snapshot(self) -> Dict[str, Any]:
        with self._monitor_lock:
            devices = sorted(self._monitor_state.keys())
            states = {str(k): v for k, v in self._monitor_state.items()}
            events = list(self._monitor_events)
        return {
            "devices": devices,
            "states": states,
            "events": events,
            "allowed_device_ids": sorted(self._allowed_device_ids) if self._allowed_device_ids else [],
        }

    def _assert_device_allowed(self, device_id: int) -> None:
        if self._allowed_device_ids is None:
            return
        if device_id not in self._allowed_device_ids:
            raise ValueError(
                f"device_id={device_id} not found in config: {self.config_path}. "
                f"Allowed ids: {sorted(self._allowed_device_ids)}"
            )

    def _get_runtime(self, device_id: int) -> DeviceRuntime:
        self._assert_device_allowed(device_id)
        with self._global_lock:
            runtime = self._runtimes.get(device_id)
            if runtime is not None:
                return runtime

            self._ensure_device_awake(device_id)

            device = AGTDevice(device_id=device_id, adb_host=self.adb_host)
            last_err = None
            for _ in range(3):
                try:
                    device.connect()
                    last_err = None
                    break
                except Exception as e:
                    last_err = e
                    time.sleep(2)
            if last_err is not None:
                raise RuntimeError(f"device_id={device_id} connect failed after auto-start: {last_err}")
            runtime = DeviceRuntime(device=device)
            self._runtimes[device_id] = runtime
            logger.info(f"Initialized device runtime for device_id={device_id}")
            return runtime

    def _handle_reset(self, runtime: DeviceRuntime, payload: Dict[str, Any]) -> Dict[str, Any]:
        task = payload.get("task", {}) or {}
        runtime.task = task
        runtime.step_count = 0
        runtime.max_steps = int(payload.get("max_steps", task.get("max_steps", 30)))

        try:
            runtime.device.key_home()
        except Exception:
            pass

        session_name = f"redroid_ws_d{runtime.device.device_id}_{int(time.time() * 1000)}"
        runtime.trajectory_path = runtime.device.init_trajectory_recording(
            output_dir=self.trajectory_root,
            session_name=session_name,
        )

        focus = runtime.device.get_focus()
        image = runtime.device.screenshot()
        prompt = str(task.get("prompt", ""))

        info = {
            "data_source": task.get("data_source", "redroid"),
            "task_id": task.get("task_id", f"task-{runtime.device.device_id}"),
            "task_prompt": prompt,
            "milestones": [],
            "native_system_prompt": SYSTEM_PROMPT,
            "trajectory_path": runtime.trajectory_path,
            "won": 0.0,
            "tool_calling": 0.0,
            "milestone_events": [],
            "is_action_valid": 1.0,
            "executor_status": "reset",
        }
        return {
            "ok": True,
            "observation_text": f"Task:\n{prompt}\n\nCurrentFocus:\n{focus.get('raw', '')}",
            "image_b64": _encode_image_to_b64(image),
            "info": info,
        }

    def _handle_step(self, runtime: DeviceRuntime, payload: Dict[str, Any]) -> Dict[str, Any]:
        runtime.step_count += 1

        action_text = str(payload.get("action_text", ""))
        record_video = bool(payload.get("record_video", False))

        status, _video, details = runtime.device.agent_call(
            action_text,
            get_video=record_video,
            return_details=True,
        )

        image = runtime.device.screenshot()
        focus = runtime.device.get_focus()

        tool_returns = details.get("tool_returns", []) if isinstance(details, dict) else []
        evidence_paths = details.get("evidence_paths", []) if isinstance(details, dict) else []

        evidence_caption_by_path: dict[str, str] = {}
        for tr in tool_returns:
            if tr.get("func_name") not in {"capture_evidence", "cature_evidence"}:
                continue
            path = tr.get("value")
            if not isinstance(path, str) or not path:
                continue
            caption = ""
            action = tr.get("action")
            if isinstance(action, dict):
                label = str(action.get("label", ""))
                m = re.search(r"caption\s*=\s*['\"](.*?)['\"]", label)
                if m:
                    caption = m.group(1)
            evidence_caption_by_path[path] = caption

        milestone_events = [
            {
                "path": path,
                "caption": evidence_caption_by_path.get(path, ""),
            }
            for path in evidence_paths
            if isinstance(path, str) and path
        ]

        has_error = str(status).lower().startswith("error:") or str(status).lower() in {"parse error", "json formatting error"}
        done_by_action = str(status).startswith("DONE:")
        done_by_limit = runtime.step_count >= runtime.max_steps
        done = bool(done_by_action or done_by_limit)

        won = 1.0 if done_by_action else 0.0
        reward = won

        traj_json_path = runtime.trajectory_path
        if done:
            finished = runtime.device.finish_trajectory_recording()
            if finished:
                traj_json_path = finished
            runtime.trajectory_path = traj_json_path

        info = {
            "won": won,
            "tool_calling": float(len(tool_returns)),
            "is_action_valid": float(not has_error),
            "tool_returns": tool_returns,
            "milestone_events": milestone_events,
            "trajectory_path": traj_json_path,
            "task_id": runtime.task.get("task_id"),
            "task_prompt": runtime.task.get("prompt"),
            "milestones": [],
            "native_system_prompt": SYSTEM_PROMPT,
            "data_source": runtime.task.get("data_source", "redroid"),
            "executor_status": status,
        }

        return {
            "ok": True,
            "observation_text": f"LastStatus: {status}\nCurrentFocus:\n{focus.get('raw', '')}",
            "image_b64": _encode_image_to_b64(image),
            "reward": reward,
            "done": done,
            "info": info,
        }

    def _handle_close(self, runtime: DeviceRuntime, payload: Dict[str, Any]) -> Dict[str, Any]:
        _ = payload
        try:
            finished = runtime.device.finish_trajectory_recording()
            if finished:
                runtime.trajectory_path = finished
        except Exception:
            pass
        return {"ok": True, "closed": True}

    def _health_check_device(
        self,
        device_id: int,
        *,
        include_screenshot: bool = False,
        include_mock_ops: bool = False,
    ) -> Dict[str, Any]:
        report: Dict[str, Any] = {
            "device_id": int(device_id),
            "ok": True,
            "checks": {},
            "errors": [],
            "timing_ms": {},
        }

        try:
            self._assert_device_allowed(device_id)
            report["checks"]["allowed"] = True
        except Exception as e:
            report["checks"]["allowed"] = False
            report["errors"].append(f"allowed_check_failed: {e}")
            report["ok"] = False
            return report

        cfg = self._device_cfg_by_id.get(device_id)
        if cfg is not None:
            try:
                t0 = time.time()
                report["checks"]["container_exists"] = self._docker_container_exists(cfg.container_name)
                report["checks"]["container_running"] = self._docker_container_running(cfg.container_name)
                report["timing_ms"]["container_probe"] = int((time.time() - t0) * 1000)
            except Exception as e:
                report["checks"]["container_probe"] = False
                report["errors"].append(f"container_probe_failed: {e}")
                report["ok"] = False

        runtime = None
        try:
            t0 = time.time()
            runtime = self._get_runtime(device_id)
            report["checks"]["runtime_ready"] = True
            report["timing_ms"]["runtime_ready"] = int((time.time() - t0) * 1000)
        except Exception as e:
            report["checks"]["runtime_ready"] = False
            report["errors"].append(f"runtime_ready_failed: {e}")
            report["ok"] = False
            return report

        assert runtime is not None
        with runtime.lock:
            try:
                t0 = time.time()
                focus = runtime.device.get_focus()
                raw_focus = str(focus.get("raw", ""))
                report["checks"]["focus_ok"] = bool(raw_focus)
                report["focus_preview"] = raw_focus[:300]
                report["timing_ms"]["focus"] = int((time.time() - t0) * 1000)
                if not raw_focus:
                    report["errors"].append("focus_empty")
                    report["ok"] = False
            except Exception as e:
                report["checks"]["focus_ok"] = False
                report["errors"].append(f"focus_failed: {e}")
                report["ok"] = False

            if include_screenshot:
                try:
                    t0 = time.time()
                    img = runtime.device.screenshot()
                    report["checks"]["screenshot_ok"] = True
                    report["screenshot_size"] = [int(img.width), int(img.height)]
                    report["timing_ms"]["screenshot"] = int((time.time() - t0) * 1000)
                except Exception as e:
                    report["checks"]["screenshot_ok"] = False
                    report["errors"].append(f"screenshot_failed: {e}")
                    report["ok"] = False

            if include_mock_ops:
                try:
                    t0 = time.time()
                    runtime.device.key_home()
                    time.sleep(0.2)
                    _ = runtime.device.get_focus()
                    report["checks"]["mock_key_home_ok"] = True
                    report["timing_ms"]["mock_key_home"] = int((time.time() - t0) * 1000)
                except Exception as e:
                    report["checks"]["mock_key_home_ok"] = False
                    report["errors"].append(f"mock_key_home_failed: {e}")
                    report["ok"] = False

        return report

    def health_check(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        include_screenshot = bool(payload.get("include_screenshot", False))
        include_mock_ops = bool(payload.get("include_mock_ops", False))

        if payload.get("device_id") is not None:
            device_ids = [int(payload.get("device_id"))]
        elif self._allowed_device_ids is not None:
            device_ids = sorted(self._allowed_device_ids)
        else:
            device_ids = sorted(self._runtimes.keys())

        reports = [
            self._health_check_device(
                did,
                include_screenshot=include_screenshot,
                include_mock_ops=include_mock_ops,
            )
            for did in device_ids
        ]

        return {
            "ok": all(bool(r.get("ok", False)) for r in reports),
            "service": {
                "adb_host": self.adb_host,
                "trajectory_root": self.trajectory_root,
                "config_path": self.config_path,
                "allowed_device_ids": sorted(self._allowed_device_ids) if self._allowed_device_ids else [],
                "runtime_count": len(self._runtimes),
            },
            "reports": reports,
        }

    def process(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        op = str(payload.get("op", "")).lower().strip()
        if op == "monitor_state":
            return {
                "ok": True,
                "monitor_snapshot": self.monitor_snapshot(),
            }
        if op == "health_check":
            return self.health_check(payload)

        device_id = int(payload.get("device_id"))
        response: Dict[str, Any]
        try:
            runtime = self._get_runtime(device_id)
            with runtime.lock:
                if op == "reset":
                    response = self._handle_reset(runtime, payload)
                elif op == "step":
                    response = self._handle_step(runtime, payload)
                elif op == "close":
                    response = self._handle_close(runtime, payload)
                elif op == "ping":
                    response = {"ok": True, "pong": True}
                else:
                    response = {"ok": False, "error": f"unsupported op: {op}"}
        except Exception as e:
            response = {"ok": False, "error": str(e)}

        self._record_event(device_id=device_id, op=op, payload=payload, response=response)
        return response


async def serve(host: str, port: int, service: RedroidWSService):
    try:
        ws_server_mod = importlib.import_module("websockets.server")
        ws_serve = getattr(ws_server_mod, "serve")
    except Exception as e:
        raise RuntimeError("websockets package is required for Redroid WS service") from e

    async def handler(websocket):
        async for message in websocket:
            req_id = None
            try:
                payload = json.loads(message)
                req_id = payload.get("request_id")
                response = service.process(payload)
                response["request_id"] = req_id
            except Exception as e:
                response = {"ok": False, "request_id": req_id, "error": str(e)}
            await websocket.send(json.dumps(response, ensure_ascii=False))

    async with ws_serve(handler, host, port, max_size=20 * 1024 * 1024):
        logger.info(f"Redroid WS service started at ws://{host}:{port}")
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description="Unified WebSocket redroid environment service")
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--ui-port", type=int, default=18081, help="HTTP monitor UI port")
    parser.add_argument("--adb-host", type=str, default="127.0.0.1")
    parser.add_argument("--workdir", type=str, default=".", help="Working directory for relative paths")
    parser.add_argument("--config-path", type=str, default="device_config.yaml", help="Device config yaml path")
    parser.add_argument("--trajectory-root", type=str, default="logs/redroid_rollout")
    args = parser.parse_args()

    if hasattr(os, "geteuid") and os.geteuid() != 0:
        raise PermissionError("WS service must be started with sudo/root privileges")

    os.chdir(os.path.abspath(args.workdir))

    config_path = args.config_path
    if not os.path.isabs(config_path):
        config_path = os.path.abspath(os.path.join(os.getcwd(), config_path))

    trajectory_root = args.trajectory_root
    if not os.path.isabs(trajectory_root):
        trajectory_root = os.path.abspath(os.path.join(os.getcwd(), trajectory_root))

    service = RedroidWSService(
        adb_host=args.adb_host,
        trajectory_root=trajectory_root,
        config_path=config_path,
    )
    run_monitor_ui(args.host, args.ui_port, ws_port=args.port, service=service)
    asyncio.run(serve(args.host, args.port, service))


if __name__ == "__main__":
    main()
