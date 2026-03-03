from __future__ import annotations

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    ctx.device.key_back()
