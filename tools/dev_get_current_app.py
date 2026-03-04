from __future__ import annotations

from loguru import logger

from .prototype import CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.get_current_app",
    summary="Get current focused app/window info.",
    example='{"label": "dev.get_current_app()"}',
)


def execute(ctx: CallContext) -> None:
    info = ctx.device.get_focus()
    logger.info(f"Current app info: {info}")
