from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.long_tap",
    summary="Long press the center point of bbox_2d.",
    args=[
        ArgSpec(
            name="duration_ms",
            type_text="int|float",
            description="Long press duration in milliseconds.",
            required=False,
            py_types=(int, float),
            default=1000,
        )
    ],
    require_bbox=True,
    example='{"bbox_2d": [100, 200, 300, 400], "label": "touch.long_tap(800)"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.bbox:
        center = ctx.center_pixel()
        if center:
            duration = ctx.args[0] if len(ctx.args) > 0 else 1000
            ctx.device.tap([center], duration=duration)
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
