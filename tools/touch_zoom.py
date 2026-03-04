from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.zoom",
    summary="Pinch zoom using bbox diagonal endpoints.",
    args=[
        ArgSpec("duration_ms", "int|float", "Zoom duration in milliseconds.", py_types=(int, float)),
        ArgSpec("scale", "int|float", "Zoom scale factor, >1 zoom in, <1 zoom out.", py_types=(int, float)),
    ],
    require_bbox=True,
    example='{"bbox_2d": [100, 200, 200, 400], "label": "touch.zoom(500,1.2)"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.bbox and len(ctx.args) >= 2:
        duration = ctx.args[0]
        scale = ctx.args[1]
        p1 = ctx.to_pixel(ctx.bbox[0], ctx.bbox[1])
        p2 = ctx.to_pixel(ctx.bbox[2], ctx.bbox[3])
        ctx.device.pinch_zoom([p1, p2], scale=scale, duration=duration)
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
