from __future__ import annotations

from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.press_task",
    summary="Press app-switch key.",
    example='{"label": "dev.press_task()"}',
)


def execute(ctx: CallContext) -> None:
    ctx.device._adb_shell("input keyevent 187")
