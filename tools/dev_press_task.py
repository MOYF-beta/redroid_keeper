from __future__ import annotations

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    ctx.device._adb_shell("input keyevent 187")
