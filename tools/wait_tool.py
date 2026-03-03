from __future__ import annotations

import time

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if ctx.args:
        ms = ctx.args[0]
        time.sleep(ms / 1000.0)
