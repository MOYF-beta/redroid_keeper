from __future__ import annotations

from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.press_home",
    summary="Press HOME key.",
    example='{"label": "dev.press_home()"}',
)


def execute(ctx: CallContext) -> None:
    ctx.device.key_home()
