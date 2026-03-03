from __future__ import annotations

import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    center = ctx.center_pixel()
    if center:
        ctx.device.tap([center])
    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
