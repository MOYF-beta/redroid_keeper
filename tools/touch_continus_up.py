from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.continus.up",
    summary="Release active continuous touch.",
    example='{"label": "touch.continus.up()"}',
)


def execute(ctx: CallContext) -> None:
    ctx.device.up()
    object.__setattr__(ctx.device, "_last_touch_pos", None)
    object.__setattr__(ctx.device, "_is_holding", False)
    object.__setattr__(ctx.device, "_hold_locked", False)
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
