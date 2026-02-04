from __future__ import annotations

import subprocess
from dataclasses import dataclass
from io import BytesIO
from typing import Iterable, TYPE_CHECKING, Optional

import ast
import json
import math
import os
import re
import shlex
import time
import base64

from loguru import logger
if TYPE_CHECKING:
    from PIL import Image

DEFAULT_ADB_BASE_PORT = 5555
DEFAULT_ADB_HOST = "127.0.0.1"
DEFAULT_TOUCH_MARGIN_MS = 10
RECORD_BEFORE = 1  # seconds
RECORD_TIME = 5   # seconds
SYSTEM_PROMPT = '''
你是一个操作Android手机的机器人。理解用户的需求，详细分析截图含有什么元素，按钮，在多轮交互中用下面的工具操作手机以完成任务。如果重复操作了数次没有效果，说明当前操作无法实现目标，需要调整思路。当需要总结内容，确保上下滑动查看并记下全部内容。每获取一些新的信息都立刻以文本形式说出以便后面整合。

请注意考虑一些常识，例如高亮的内容代表已经选中。你应该列出常识，做出思考，再付诸行动。尽可能跳过无用的东西或是登录，尽可能选择有利于隐私的选项。

下面是你可以使用的工具：

touch工具可以通过bbox检测框操作屏幕:

- touch.tap() # 点按bbox中心
- touch.long_tap(duration_ms) # 长按bbox中心
- touch.swipe(duration_ms,direction) # direction 指定极角(度)，在bbox内过目标中心沿指定方向划动。提示：对于导航类的滑动，距离往往很大，并且自然滑动是反向的，例如向上滑动屏幕查看下方内容，实际上是手指从屏幕下方向上滑动
- touch.zoom(duration_ms,scale) # 按照scale沿bbox对角线缩放指定倍数
- touch.continus.down() # 按下屏幕直到下一次 up调用，在此期间可以用 move_to移动
- touch.continus.down(hold=True) # 按下屏幕直到下一次 up调用，该状态可以跨越多轮对话保持，适合按压后需要观察变化的场景
- touch.continus.move_to(duration_ms) # 从当前位置移动到bbox中心
- touch.continus.up() # 松开
通过label调用工具touch工具，例如
{"bbox_2d": [x, y, x, y], "label": "touch.tool_name1(args1) "}
continus状态会被普通操作打断

text工具用于文本输入，尽可能使用此工具，除非必要避免使用屏幕键盘:

- text.input(text:str) # 提示，在输入前一般需要先点击输入框以获取焦点
- text.del(n_char:int) # 删除光标前n_char个字符，默认为50

dev工具用于设备操作:

- dev.screenshot(output_path:str) (保存在./agt_screenshots/目录下)
- dev.press_home() # 此工具最适合回到桌面
- dev.press_back() # 此工具用户回到上一个页面
- dev.press_task() # 此工具用于打开多任务管理界面，连续调用两次可以回到之前的应用

wait工具用于等待:
- wait(duration_ms)

任务完成后向user返回结果:
- done(message:str) # 结束当前任务，返回message给user

请以json格式输出调用列表到代码块，工具调用一行一个，这些调用按顺序执行
为了确保的操作有效，思路清晰，请严格按照下面分割线中的模板输出:
-------------------------------------------------------------
我有已知信息: {需要记录的memo}

上一步，我进行了:{上一步的操作}，看起来效果是:{效果描述}，我{成功 or 失败}地完成了上一步。
而现在，我在屏幕上看到了：{屏幕上的各个元素}
这意味着：{当前手机的状态}{当前屏幕信息的意义}
我的最终目的是：{我的目标}
我的阶段性目的是：{当前阶段目标}
所以我应该：{我的动作}
下面是我的动作对应的工具调用:
```agent_call
[
{"bbox_2d": [100, 200, 300, 400], "label": "tool() # comment 1"},
{"bbox_2d": [400, 500, 600, 700], "label": "tool(args) # comment 2"}
]
```
-------------------------------------------------------------
'''



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

            # locate device_config.yaml: try repo root next to this package, then CWD
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

    # Internal maatouch device, created after connect()
    _touch_device: Optional["AGTDevice"] = None
    # Track holding state
    _is_holding: bool = False
    _hold_locked: bool = False

    def is_holding(self) -> bool:
        return getattr(self, "_is_holding", False)
    
    def is_hold_locked(self) -> bool:
        return getattr(self, "_hold_locked", False)

    def get_system_prompt(self) -> str:
        prompt = SYSTEM_PROMPT + "\n当前touch.continus.down()激活，你正在持续按压屏幕，只能调用 touch.continus.move_to() 和 wait()，touch.continus.up()松开"

        if self.is_hold_locked():
             prompt += '\n\n正在持续按压屏幕，只能调用 touch.continus.move_to() 和 wait()，touch.continus.up()松开'
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
    
    def agent_call(self, call_string: str, get_video: bool = False) -> str:
        logger.info(f"agent_call: {call_string}")

        path_to_video = None
        screen_record_process = None
        remote_video_path = f"/sdcard/record_{int(time.time())}.mp4"

        if get_video:
            # Start recording in the background
            record_cmd = [
                "adb", "-s", self.adb_target, "shell",
                "screenrecord", f"--time-limit={int(RECORD_BEFORE + RECORD_TIME)}", remote_video_path
            ]
            logger.debug(f"Starting screen record: {' '.join(record_cmd)}")
            screen_record_process = subprocess.Popen(record_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(RECORD_BEFORE)
        
        # 1. Extract JSON block
        json_content = call_string
        code_block_match = re.search(r"```agent_call\s*(.*?)\s*```", call_string, re.DOTALL)
        if code_block_match:
            json_content = code_block_match.group(1)
        else:
             # Fallback: try finding first [ ... ]
             json_list_match = re.search(r"(\[.*\])", call_string, re.DOTALL)
             if json_list_match:
                 json_content = json_list_match.group(1)
        
        # Replace tabs with spaces as YAML does not support tabs for indentation
        json_content = json_content.replace("\t", "    ")

        try:
            # use yaml to parse as it's more tolerant of trailing commas/comments
            import yaml
            actions = yaml.safe_load(json_content)
        except Exception as e:
            logger.error(f"Parse error: {e}")
            logger.error(f"Failed content: {json_content!r}")
            return "parse error"
            
        if not isinstance(actions, list):
            if isinstance(actions, dict):
                actions = [actions]
            else:
                logger.error(f"JSON is not a list or dict: {type(actions)}")
                return "json formatting error"

        # 2. Execute actions
        for action in actions:
            try:
                label = action.get("label", "")
                bbox = action.get("bbox_2d", None)
                
                # Strip comments
                if "#" in label:
                    label = label.split("#", 1)[0]
                label = label.strip()
                
                # Parse function call: name(args)
                func_match = re.match(r"^([\w\.]+)\((.*)\)$", label)
                if not func_match:
                    logger.warning(f"Skipping invalid label: {label}")
                    continue
                
                func_name, args_str = func_match.groups()
                
                # Parse arguments using ast
                pos_args = ()
                kw_args = {}
                if args_str.strip():
                    try:
                        # Parse as a call expression: func(args...)
                        # args_str might be "100" or "hold=True" or "100, 200"
                        # We construct a dummy call "f(args_str)" to parse it.
                        tree = ast.parse(f"f({args_str})", mode='eval')
                        call_node = tree.body
                        
                        def _safe_eval(node):
                            try:
                                return ast.literal_eval(node)
                            except ValueError:
                                if isinstance(node, ast.Name):
                                    return node.id
                                raise

                        # Evaluate args
                        pos_args = tuple(_safe_eval(a) for a in call_node.args)
                        kw_args = {k.arg: _safe_eval(k.value) for k in call_node.keywords}
                    except Exception:
                        logger.warning(f"Failed to parse args with ast: {args_str}, fallback to literal_eval")
                        try:
                             # Fallback to old behavior for simple tuples
                             pos_args = ast.literal_eval(f"({args_str},)")
                        except Exception:
                             if func_name == "text.input":
                                 # Allow raw strings for text input if parsing fails (e.g. spaces, special chars)
                                 pos_args = (args_str,)
                             else:
                                 logger.warning(f"Failed to parse args completely: {args_str}")
                                 continue

                # Merege args for legacy compatibility (some code uses numerical indexing)
                args = pos_args 

                logger.debug(f"Executing {func_name} with pos_args={pos_args} kw_args={kw_args} bbox={bbox}")

                # -------------------------------------------------------------
                # Strict Checking for Hold State
                # -------------------------------------------------------------
                if self.is_hold_locked():
                    allowed_funcs = ["touch.continus.move_to", "touch.continus.up", "wait"]
                    if func_name not in allowed_funcs:
                        raise RuntimeError(f"Constraint Violation: Device is in hold state. Only {allowed_funcs} are allowed. Got {func_name}")

                # Auto-release for non-continuous actions (except wait)
                if getattr(self, "_last_touch_pos", None) and not func_name.startswith("touch.continus") and func_name != "wait":
                    self.up()
                    object.__setattr__(self, "_last_touch_pos", None)
                    object.__setattr__(self, "_is_holding", False)
                    object.__setattr__(self, "_hold_locked", False)

                # Coordinate Conversion Helper
                def to_pixel(rel_x, rel_y):
                    return int(rel_x * self.width / 1000), int(rel_y * self.height / 1000)

                # -------------------------------------------------------------
                # Implicit Tap Logic & Touch Delay
                # -------------------------------------------------------------
                # If a bbox is provided but the function is NOT a touch consumer (e.g. text.input),
                # perform an implicit tap on the center of the bbox first.
                touch_consumers = {
                    "touch.tap", "touch.long_tap", "touch.swipe", "touch.zoom", 
                    "touch.continus.down", "touch.continus.move_to"
                }
                
                if bbox and func_name not in touch_consumers:
                    cx = (bbox[0] + bbox[2]) / 2
                    cy = (bbox[1] + bbox[3]) / 2
                    px, py = to_pixel(cx, cy)
                    logger.debug(f"Implicit tap at {px},{py} for {func_name}")
                    self.tap([(px, py)])
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)

                if func_name == "touch.tap":
                    if bbox:
                        cx = (bbox[0] + bbox[2]) / 2
                        cy = (bbox[1] + bbox[3]) / 2
                        px, py = to_pixel(cx, cy)
                        self.tap([(px, py)])
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
                
                elif func_name == "touch.long_tap":
                    if bbox:
                        duration = args[0] if len(args) > 0 else 1000
                        cx = (bbox[0] + bbox[2]) / 2
                        cy = (bbox[1] + bbox[3]) / 2
                        px, py = to_pixel(cx, cy)
                        self.tap([(px, py)], duration=duration)
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
                
                elif func_name == "touch.swipe":
                    if bbox and len(args) >= 2:
                        duration = args[0]
                        direction = str(args[1]).lower()
                        
                        angle_map = {
                            "up": 90,
                            "down": 270,
                            "left": 180,
                            "right": 0
                        }
                        
                        if direction in angle_map:
                            angle = angle_map[direction]
                        else:
                            try:
                                angle = float(direction)
                            except ValueError:
                                angle = 0
                        
                        # bbox is [x1, y1, x2, y2]
                        cx_raw = (bbox[0] + bbox[2]) / 2
                        cy_raw = (bbox[1] + bbox[3]) / 2
                        
                        w_rel = abs(bbox[2] - bbox[0])
                        h_rel = abs(bbox[3] - bbox[1])
                        
                        # Angle to vector
                        rad = math.radians(angle)
                        # direction vector
                        v_x = math.cos(rad)
                        v_y = -math.sin(rad)
                        
                        # Calculate distance to edge (half length of the segment)
                        # We want to find t such that (cx + t*vx, cy + t*vy) is on the box boundary
                        # The boundaries are defined by +/- w_rel/2 and +/- h_rel/2 relative to center
                        
                        LIMIT_X = w_rel / 2
                        LIMIT_Y = h_rel / 2
                        
                        # Avoid division by zero
                        # t_x is the distance to travel along V to hit X boundary
                        if abs(v_x) > 1e-9:
                            t_x = abs(LIMIT_X / v_x)
                        else:
                            t_x = float('inf')
                            
                        # t_y is the distance to travel along V to hit Y boundary
                        if abs(v_y) > 1e-9:
                            t_y = abs(LIMIT_Y / v_y)
                        else:
                            t_y = float('inf')
                        
                        # The intersection is limited by the tighter boundary
                        t = min(t_x, t_y)
                        
                        # Full segment vector relative to center is +/- (t * V)
                        dx = t * v_x
                        dy = t * v_y
                        
                        # Start from one side, go to the other
                        start_px_py = to_pixel(cx_raw - dx, cy_raw - dy)
                        end_px_py = to_pixel(cx_raw + dx, cy_raw + dy)
                        
                        # Use ext_smooth_swipe for better simulation
                        # Calculate parts for ~50Hz (20ms) updates if possible
                        part = max(int(duration / 20), 5)
                        self.ext_smooth_swipe([start_px_py, end_px_py], duration=duration, part=part)
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
                        
                elif func_name == "touch.zoom":
                    if bbox and len(args) >= 2:
                        duration = args[0]
                        scale = args[1]
                        
                        # Zoom uses two points. Diagonal corners
                        p1 = to_pixel(bbox[0], bbox[1])
                        p2 = to_pixel(bbox[2], bbox[3])
                        self.pinch_zoom([p1, p2], scale=scale, duration=duration)
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
                
                elif func_name == "touch.continus.down":
                    if bbox:
                        if getattr(self, "_last_touch_pos", None):
                             self.up()
                             object.__setattr__(self, "_is_holding", False)
                             object.__setattr__(self, "_hold_locked", False)

                        cx = (bbox[0] + bbox[2]) / 2
                        cy = (bbox[1] + bbox[3]) / 2
                        px, py = to_pixel(cx, cy)
                        self.tap([(px, py)], no_up=True)
                        object.__setattr__(self, "_last_touch_pos", (px, py))
                        object.__setattr__(self, "_is_holding", True)

                        # Check Hold param
                        hold_val = kw_args.get("hold")
                        if hold_val is None and len(args) > 0:
                             if isinstance(args[0], bool):
                                 hold_val = args[0]
                        
                        if hold_val is True:
                             object.__setattr__(self, "_hold_locked", True)
                        else:
                             object.__setattr__(self, "_hold_locked", False)
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)

                elif func_name == "touch.continus.move_to":
                    if bbox:
                         start_pos = getattr(self, "_last_touch_pos", None)
                         if not start_pos:
                             logger.warning("touch.continus.move_to called without active touch")
                         else:
                             cx = (bbox[0] + bbox[2]) / 2
                             cy = (bbox[1] + bbox[3]) / 2
                             px, py = to_pixel(cx, cy)
                             
                             duration = args[0] if len(args) > 0 else 500
                             self.ext_smooth_swipe([start_pos, (px, py)], duration=duration, no_down=True, no_up=True)
                             object.__setattr__(self, "_last_touch_pos", (px, py))
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)

                elif func_name == "touch.continus.up":
                    self.up()
                    object.__setattr__(self, "_last_touch_pos", None)
                    object.__setattr__(self, "_is_holding", False)
                    object.__setattr__(self, "_hold_locked", False)
                    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)

                elif func_name == "text.input":
                    if args:
                        text_content = str(args[0])
                        safe_text = shlex.quote(text_content)
                        is_ascii = text_content.isascii()
                        if is_ascii:
                            self._adb_shell(f"input text {safe_text}")
                        else:
                            # Use ADBKeyBoard broadcast for non-ASCII (e.g. Chinese)
                            # Requires ADBKeyBoard to be installed and enabled
                            # Use base64 to avoid shell encoding issues
                            b64_encoded = base64.b64encode(text_content.encode('utf-8')).decode('utf-8')
                            self._adb_shell(f"am broadcast -a ADB_INPUT_B64 --es msg {b64_encoded}", check=False)

                elif func_name == "text.del" or func_name == "text.clear":
                    # Delete n_char chars
                    if func_name == "text.del" and args:
                        n_char = int(args[0])
                    else:
                        n_char = 1
                        
                    cmd = "input keyevent 123" # MOVE_END
                    # Split into chunks to avoid "Argument list too long" if n_char is large
                    # Max length of command line is limited, but here we concatenate many commands.
                    # It's safer to loop if n_char is very large, but forreasonable sizes:
                    
                    if n_char > 100:
                         # Process in batches
                         for _ in range(n_char):
                             self._adb_shell("input keyevent 67")
                    else:
                        for _ in range(n_char):
                             cmd += "; input keyevent 67" # DEL
                        self._adb_shell(cmd)

                elif func_name == "dev.screenshot":
                    if args:
                        path = args[0]
                        dir_path = os.path.dirname(os.path.abspath(path))
                        if dir_path:
                            os.makedirs(dir_path, exist_ok=True)
                        img = self.screenshot()
                        img.save(path)
                        logger.info(f"Screenshot saved to {path}")

                elif func_name == "dev.get_current_app":
                    info = self.get_focus()
                    logger.info(f"Current app info: {info}")

                elif func_name == "dev.launch_app":
                    if args:
                        self.launch_app(args[0])

                elif func_name == "dev.press_home":
                    self.key_home()

                elif func_name == "dev.press_back":
                    self.key_back()

                elif func_name == "dev.press_task":
                    # KEYCODE_APP_SWITCH = 187
                    self._adb_shell("input keyevent 187")

                elif func_name == "wait":
                    if args:
                        ms = args[0]
                        time.sleep(ms / 1000.0)

                elif func_name == "done":
                    msg = args[0] if args else "task completed"
                    logger.info(f"Task Done: {msg}")
                    return f"DONE: {msg}"

                else:
                    logger.warning(f"Unknown function: {func_name}")

            except Exception as e:
                # If constraint violation, re-raise to fail the call explicitly
                if "Constraint Violation" in str(e):
                    raise e
                logger.error(f"Error executing action {action}: {e}")
        
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

        return "success", path_to_video


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
