from __future__ import annotations

from .prototype import ArgSpec, CallContext, CallResult, MethodSpec


METHOD_SPEC = MethodSpec(
    name="done",
    summary="Finish task and return final message.",
    args=[ArgSpec("message", "str", "Final response message.", required=False, py_types=(str,), default="task completed")],
    example='{"label": "done(\"已完成\")"}',
)


def execute(ctx: CallContext) -> CallResult:
    msg = ctx.args[0] if ctx.args else "task completed"
    return CallResult(ok=True, done=True, value=str(msg))
