from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if ctx.bbox:
        center = ctx.center_pixel()
        if center:
            duration = ctx.args[0] if len(ctx.args) > 0 else 1000
            ctx.device.tap([center], duration=duration)
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
