from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.continus.move_to",
    summary="Move active continuous touch to bbox center.",
    args=[
        ArgSpec("duration_ms", "int|float", "Movement duration in milliseconds.", required=False, py_types=(int, float), default=500)
    ],
    require_bbox=True,
    example='{"bbox_2d": [200, 300, 400, 500], "label": "touch.continus.move_to(500)"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.bbox:
        start_pos = getattr(ctx.device, "_last_touch_pos", None)
        if start_pos:
            center = ctx.center_pixel()
            if center:
                px, py = center
                duration = ctx.args[0] if len(ctx.args) > 0 else 500
                ctx.device.ext_smooth_swipe([start_pos, (px, py)], duration=duration, no_down=True, no_up=True)
                object.__setattr__(ctx.device, "_last_touch_pos", (px, py))
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
