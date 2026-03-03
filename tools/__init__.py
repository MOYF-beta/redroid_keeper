from __future__ import annotations

from typing import Callable

from .prototype import CallContext
from .capture_evidence import execute as capture_evidence
from .dev_get_current_app import execute as dev_get_current_app
from .dev_launch_app import execute as dev_launch_app
from .dev_press_back import execute as dev_press_back
from .dev_press_home import execute as dev_press_home
from .dev_press_task import execute as dev_press_task
from .dev_screenshot import execute as dev_screenshot
from .done_tool import execute as done_tool
from .text_delete import execute as text_delete
from .text_input import execute as text_input
from .touch_continus_down import execute as touch_continus_down
from .touch_continus_move_to import execute as touch_continus_move_to
from .touch_continus_up import execute as touch_continus_up
from .touch_long_tap import execute as touch_long_tap
from .touch_swipe import execute as touch_swipe
from .touch_tap import execute as touch_tap
from .touch_zoom import execute as touch_zoom
from .wait_tool import execute as wait_tool

ToolHandler = Callable[[CallContext], object]

TOOL_REGISTRY: dict[str, ToolHandler] = {
    "touch.tap": touch_tap,
    "touch.long_tap": touch_long_tap,
    "touch.swipe": touch_swipe,
    "touch.zoom": touch_zoom,
    "touch.continus.down": touch_continus_down,
    "touch.continus.move_to": touch_continus_move_to,
    "touch.continus.up": touch_continus_up,
    "text.input": text_input,
    "text.del": text_delete,
    "text.clear": text_delete,
    "dev.screenshot": dev_screenshot,
    "cature_evidence": capture_evidence,
    "capture_evidence": capture_evidence,
    "dev.get_current_app": dev_get_current_app,
    "dev.launch_app": dev_launch_app,
    "dev.press_home": dev_press_home,
    "dev.press_back": dev_press_back,
    "dev.press_task": dev_press_task,
    "wait": wait_tool,
    "done": done_tool,
}


def get_tool_handler(func_name: str) -> ToolHandler | None:
    return TOOL_REGISTRY.get(func_name)
