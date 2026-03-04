from __future__ import annotations

from .prototype import ToolEntry
from .capture_evidence import METHOD_SPEC as capture_evidence_spec, execute as capture_evidence
from .dev_get_current_app import METHOD_SPEC as dev_get_current_app_spec, execute as dev_get_current_app
from .dev_launch_app import METHOD_SPEC as dev_launch_app_spec, execute as dev_launch_app
from .dev_press_back import METHOD_SPEC as dev_press_back_spec, execute as dev_press_back
from .dev_press_home import METHOD_SPEC as dev_press_home_spec, execute as dev_press_home
from .dev_press_task import METHOD_SPEC as dev_press_task_spec, execute as dev_press_task
from .dev_screenshot import METHOD_SPEC as dev_screenshot_spec, execute as dev_screenshot
from .done_tool import METHOD_SPEC as done_tool_spec, execute as done_tool
from .text_delete import METHOD_SPEC as text_delete_spec, execute as text_delete
from .text_input import METHOD_SPEC as text_input_spec, execute as text_input
from .touch_continus_down import METHOD_SPEC as touch_continus_down_spec, execute as touch_continus_down
from .touch_continus_move_to import METHOD_SPEC as touch_continus_move_to_spec, execute as touch_continus_move_to
from .touch_continus_up import METHOD_SPEC as touch_continus_up_spec, execute as touch_continus_up
from .touch_long_tap import METHOD_SPEC as touch_long_tap_spec, execute as touch_long_tap
from .touch_swipe import METHOD_SPEC as touch_swipe_spec, execute as touch_swipe
from .touch_tap import METHOD_SPEC as touch_tap_spec, execute as touch_tap
from .touch_zoom import METHOD_SPEC as touch_zoom_spec, execute as touch_zoom
from .wait_tool import METHOD_SPEC as wait_tool_spec, execute as wait_tool

TOOL_REGISTRY: dict[str, ToolEntry] = {
    "touch.tap": ToolEntry(handler=touch_tap, spec=touch_tap_spec),
    "touch.long_tap": ToolEntry(handler=touch_long_tap, spec=touch_long_tap_spec),
    "touch.swipe": ToolEntry(handler=touch_swipe, spec=touch_swipe_spec),
    "touch.zoom": ToolEntry(handler=touch_zoom, spec=touch_zoom_spec),
    "touch.continus.down": ToolEntry(handler=touch_continus_down, spec=touch_continus_down_spec),
    "touch.continus.move_to": ToolEntry(handler=touch_continus_move_to, spec=touch_continus_move_to_spec),
    "touch.continus.up": ToolEntry(handler=touch_continus_up, spec=touch_continus_up_spec),
    "text.input": ToolEntry(handler=text_input, spec=text_input_spec),
    "text.del": ToolEntry(handler=text_delete, spec=text_delete_spec),
    "text.clear": ToolEntry(handler=text_delete, spec=text_delete_spec),
    "dev.screenshot": ToolEntry(handler=dev_screenshot, spec=dev_screenshot_spec),
    "cature_evidence": ToolEntry(handler=capture_evidence, spec=capture_evidence_spec),
    "capture_evidence": ToolEntry(handler=capture_evidence, spec=capture_evidence_spec),
    "dev.get_current_app": ToolEntry(handler=dev_get_current_app, spec=dev_get_current_app_spec),
    "dev.launch_app": ToolEntry(handler=dev_launch_app, spec=dev_launch_app_spec),
    "dev.press_home": ToolEntry(handler=dev_press_home, spec=dev_press_home_spec),
    "dev.press_back": ToolEntry(handler=dev_press_back, spec=dev_press_back_spec),
    "dev.press_task": ToolEntry(handler=dev_press_task, spec=dev_press_task_spec),
    "wait": ToolEntry(handler=wait_tool, spec=wait_tool_spec),
    "done": ToolEntry(handler=done_tool, spec=done_tool_spec),
}


def get_tool_entry(func_name: str) -> ToolEntry | None:
    return TOOL_REGISTRY.get(func_name)


def get_tool_handler(func_name: str):
    entry = TOOL_REGISTRY.get(func_name)
    return entry.handler if entry else None
