from __future__ import annotations

from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.press_back",
    summary="Press BACK key.",
    example='{"label": "dev.press_back()"}',
)


def execute(ctx: CallContext) -> None:
    ctx.device.key_back()
