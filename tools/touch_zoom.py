from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if ctx.bbox and len(ctx.args) >= 2:
        duration = ctx.args[0]
        scale = ctx.args[1]
        p1 = ctx.to_pixel(ctx.bbox[0], ctx.bbox[1])
        p2 = ctx.to_pixel(ctx.bbox[2], ctx.bbox[3])
        ctx.device.pinch_zoom([p1, p2], scale=scale, duration=duration)
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
