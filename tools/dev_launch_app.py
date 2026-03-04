from __future__ import annotations

from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.launch_app",
    summary="Launch app by package name.",
    args=[ArgSpec("package_name", "str", "Android package name.", py_types=(str,))],
    example='{"label": "dev.launch_app(\"com.tencent.mm\")"}',
)


def execute(ctx: CallContext) -> None:
    if ctx.args:
        ctx.device.launch_app(str(ctx.args[0]))
