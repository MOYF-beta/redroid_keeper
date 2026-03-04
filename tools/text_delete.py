from __future__ import annotations

from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="text.del|text.clear",
    summary="Delete characters before cursor.",
    args=[ArgSpec("n_char", "int", "Number of chars to delete.", required=False, py_types=(int,), default=1)],
    example='{"label": "text.del(10)"}',
)


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
