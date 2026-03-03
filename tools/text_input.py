from __future__ import annotations

import base64
import re
import shlex
import time

from ..constants import (
    DEFAULT_KEY_INPUT_MARGIN_MS,
    DEFAULT_PRE_INPUT_DELAY_MS,
    DEFAULT_TOUCH_MARGIN_MS,
)
from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if not ctx.args:
        return

    if ctx.bbox and not ctx.did_implicit_tap:
        center = ctx.center_pixel()
        if center:
            ctx.device.tap([center])
            time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)

    if ctx.bbox:
        time.sleep(DEFAULT_PRE_INPUT_DELAY_MS / 1000.0)

    text_content = str(ctx.args[0])
    parts = re.split(r"(\\t|\\n|\t|\n)", text_content)
    for part in parts:
        if part in ("\t", "\\t"):
            ctx.device._adb_shell("input keyevent KEYCODE_TAB")
            time.sleep(DEFAULT_KEY_INPUT_MARGIN_MS / 1000.0)
            continue
        if part in ("\n", "\\n"):
            ctx.device._adb_shell("input keyevent KEYCODE_ENTER")
            time.sleep(DEFAULT_KEY_INPUT_MARGIN_MS / 1000.0)
            continue
        if not part:
            continue

        safe_text = shlex.quote(part)
        if part.isascii():
            ctx.device._adb_shell(f"input text {safe_text}")
            time.sleep(DEFAULT_KEY_INPUT_MARGIN_MS / 1000.0)
        else:
            b64_encoded = base64.b64encode(part.encode("utf-8")).decode("utf-8")
            ctx.device._adb_shell(f"am broadcast -a ADB_INPUT_B64 --es msg {b64_encoded}", check=False)
            time.sleep(DEFAULT_KEY_INPUT_MARGIN_MS / 1000.0)
