from __future__ import annotations

from loguru import logger

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    info = ctx.device.get_focus()
    logger.info(f"Current app info: {info}")
