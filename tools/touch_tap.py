from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="touch.tap",
    summary="Tap the center point of bbox_2d.",
    require_bbox=True,
    example='{"bbox_2d": [100, 200, 300, 400], "label": "touch.tap()"}',
)


def execute(ctx: CallContext) -> None:
    center = ctx.center_pixel()
    if center:
        ctx.device.tap([center])
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
