from __future__ import annotations

import os

from .prototype import ArgSpec, CallContext, MethodSpec


METHOD_SPEC = MethodSpec(
    name="dev.screenshot",
    summary="Save screenshot to local path.",
    args=[ArgSpec("output_path", "str", "Output image path.", py_types=(str,))],
    example='{"label": "dev.screenshot(\"logs/screen.png\")"}',
)


def execute(ctx: CallContext) -> None:
    if not ctx.args:
        return None

    path = str(ctx.args[0])
    dir_path = os.path.dirname(os.path.abspath(path))
    if dir_path:
        os.makedirs(dir_path, exist_ok=True)

    img = ctx.device.screenshot()
    img.save(path)
    return path
