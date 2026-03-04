from __future__ import annotations

import math
import time

from ..constants import DEFAULT_TOUCH_MARGIN_MS
from .prototype import ArgSpec, CallContext, MethodSpec, ToolValidationError


def _validate_swipe(ctx: CallContext) -> None:
    duration = ctx.args[0] if len(ctx.args) >= 1 else ctx.kw_args.get("duration_ms")
    direction = ctx.args[1] if len(ctx.args) >= 2 else ctx.kw_args.get("direction")
    if duration is None or direction is None:
        raise ToolValidationError("MissingRequiredArgument", "touch.swipe requires duration_ms and direction")
    if not isinstance(duration, (int, float)):
        raise ToolValidationError("InvalidArgumentType", f"duration_ms expects int|float, got {type(duration).__name__}")
    if not isinstance(direction, (int, float, str)):
        raise ToolValidationError("InvalidArgumentType", f"direction expects int|float|str, got {type(direction).__name__}")


METHOD_SPEC = MethodSpec(
    name="touch.swipe",
    summary="Swipe across bbox center with angle direction.",
    args=[
        ArgSpec("duration_ms", "int|float", "Swipe duration in milliseconds.", required=False, py_types=(int, float)),
        ArgSpec("direction", "int|float|str", "Direction angle in degrees or up/down/left/right.", required=False, py_types=(int, float, str)),
    ],
    kwargs={
        "duration_ms": ArgSpec("duration_ms", "int|float", "Swipe duration in milliseconds.", required=False, py_types=(int, float)),
        "direction": ArgSpec("direction", "int|float|str", "Direction angle in degrees or up/down/left/right.", required=False, py_types=(int, float, str)),
    },
    require_bbox=True,
    example='{"bbox_2d": [0, 0, 1000, 1000], "label": "touch.swipe(duration_ms=500,direction=90)"}',
    custom_validator=_validate_swipe,
)


def execute(ctx: CallContext) -> None:
    if ctx.bbox:
        duration = None
        direction = None
        if len(ctx.args) >= 2:
            duration = ctx.args[0]
            direction = ctx.args[1]
        else:
            duration = ctx.kw_args.get("duration_ms")
            direction = ctx.kw_args.get("direction")

        if duration is None or direction is None:
            time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
            return

        direction = str(direction).lower()
        angle_map = {"up": 90, "down": 270, "left": 180, "right": 0}

        if direction in angle_map:
            angle = angle_map[direction]
        else:
            try:
                angle = float(direction)
            except ValueError:
                angle = 0

        cx_raw = (ctx.bbox[0] + ctx.bbox[2]) / 2
        cy_raw = (ctx.bbox[1] + ctx.bbox[3]) / 2
        w_rel = abs(ctx.bbox[2] - ctx.bbox[0])
        h_rel = abs(ctx.bbox[3] - ctx.bbox[1])

        rad = math.radians(angle)
        v_x = math.cos(rad)
        v_y = -math.sin(rad)

        limit_x = w_rel / 2
        limit_y = h_rel / 2

        t_x = abs(limit_x / v_x) if abs(v_x) > 1e-9 else float("inf")
        t_y = abs(limit_y / v_y) if abs(v_y) > 1e-9 else float("inf")
        t = min(t_x, t_y)

        dx = t * v_x
        dy = t * v_y

        start_px_py = ctx.to_pixel(cx_raw - dx, cy_raw - dy)
        end_px_py = ctx.to_pixel(cx_raw + dx, cy_raw + dy)

        part = max(int(duration / 20), 5)
        ctx.device.ext_smooth_swipe([start_px_py, end_px_py], duration=duration, part=part)
        hold_ms = max(int(duration * 0.1), 1)
        time.sleep(hold_ms / 1000.0)

    time.sleep(DEFAULT_TOUCH_MARGIN_MS / 1000.0)
