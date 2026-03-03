from __future__ import annotations

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if ctx.func_name == "text.del" and ctx.args:
        n_char = int(ctx.args[0])
    else:
        n_char = 1

    cmd = "input keyevent 123"
    if n_char > 100:
        for _ in range(n_char):
            ctx.device._adb_shell("input keyevent 67")
    else:
        for _ in range(n_char):
            cmd += "; input keyevent 67"
        ctx.device._adb_shell(cmd)
