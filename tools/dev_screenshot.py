from __future__ import annotations

import os

from .prototype import CallContext


def execute(ctx: CallContext) -> None:
    if not ctx.args:
        return

    path = str(ctx.args[0])
    dir_path = os.path.dirname(os.path.abspath(path))
    if dir_path:
        os.makedirs(dir_path, exist_ok=True)

    img = ctx.device.screenshot()
    img.save(path)
