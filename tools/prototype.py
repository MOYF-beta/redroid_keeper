from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class CallResult:
    ok: bool
    value: Any = None
    done: bool = False
    error_message: str | None = None
    exception: Exception | None = None


@dataclass
class CallContext:
    device: Any
    action: dict[str, Any]
    func_name: str
    bbox: list[float] | None
    args: tuple[Any, ...]
    kw_args: dict[str, Any]
    did_implicit_tap: bool = False
    extras: dict[str, Any] = field(default_factory=dict)

    def to_pixel(self, rel_x: float, rel_y: float) -> tuple[int, int]:
        return int(rel_x * self.device.width / 1000), int(rel_y * self.device.height / 1000)

    def center_pixel(self) -> tuple[int, int] | None:
        if not self.bbox or len(self.bbox) != 4:
            return None
        cx = (self.bbox[0] + self.bbox[2]) / 2
        cy = (self.bbox[1] + self.bbox[3]) / 2
        return self.to_pixel(cx, cy)


def build_error_message(ctx: CallContext, exc: Exception) -> str:
    label = str(ctx.action.get("label", ""))
    bbox = ctx.action.get("bbox_2d", None)
    return (
        f"tool call failed: {ctx.func_name}; "
        f"label={label!r}; bbox={bbox}; "
        f"args={ctx.args}; kwargs={ctx.kw_args}; "
        f"error={type(exc).__name__}: {exc}"
    )


def call(ctx: CallContext, impl: Callable[[CallContext], Any]) -> CallResult:
    try:
        result = impl(ctx)
        if isinstance(result, CallResult):
            return result
        return CallResult(ok=True, value=result)
    except Exception as exc:
        return CallResult(
            ok=False,
            error_message=build_error_message(ctx, exc),
            exception=exc,
        )
