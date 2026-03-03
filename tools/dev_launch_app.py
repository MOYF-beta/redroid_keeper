from __future__ import annotations

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if ctx.args:
        ctx.device.launch_app(str(ctx.args[0]))
