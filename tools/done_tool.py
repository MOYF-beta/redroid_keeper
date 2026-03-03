from __future__ import annotations

from .prototype import CallContext, CallResult


def execute(ctx: CallContext) -> CallResult:
    msg = ctx.args[0] if ctx.args else "task completed"
    return CallResult(ok=True, done=True, value=str(msg))
