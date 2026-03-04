from __future__ import annotations

import time

from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="wait",
    summary="Sleep for given duration.",
    args=[ArgSpec("duration_ms", "int|float", "Wait milliseconds.", required=False, py_types=(int, float), default=0)],
    example='{"label": "wait(1000)"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.args:
        ms = ctx.args[0]
        time.sleep(ms / 1000.0)
