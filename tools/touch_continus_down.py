from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.continus.down",
    summary="Press and hold at bbox center.",
    args=[
        ArgSpec("hold", "bool", "If true, allow hold state to persist across rounds.", required=False, py_types=(bool,), default=False)
    ],
    kwargs={
        "hold": ArgSpec("hold", "bool", "If true, allow hold state to persist across rounds.", required=False, py_types=(bool,), default=False)
    },
    require_bbox=True,
    example='{"bbox_2d": [100, 200, 300, 400], "label": "touch.continus.down(hold=True)"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.bbox:
        if getattr(ctx.device, "_last_touch_pos", None):
            ctx.device.up()
            object.__setattr__(ctx.device, "_is_holding", False)
            object.__setattr__(ctx.device, "_hold_locked", False)

        center = ctx.center_pixel()
        if center:
            px, py = center
            ctx.device.tap([(px, py)], no_up=True)
            object.__setattr__(ctx.device, "_last_touch_pos", (px, py))
            object.__setattr__(ctx.device, "_is_holding", True)

            hold_val = ctx.kw_args.get("hold")
            if hold_val is None and len(ctx.args) > 0 and isinstance(ctx.args[0], bool):
                hold_val = ctx.args[0]

            object.__setattr__(ctx.device, "_hold_locked", hold_val is True)

    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
