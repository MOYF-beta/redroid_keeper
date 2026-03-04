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


class ToolValidationError(Exception):
    def __init__(self, error_type: str, message: str):
        super().__init__(message)
        self.error_type = error_type
        self.message = message


@dataclass
class ArgSpec:
    name: str
    type_text: str
    description: str
    required: bool = True
    py_types: tuple[type, ...] | None = None
    default: Any = None


@dataclass
class MethodSpec:
    name: str
    summary: str
    args: list[ArgSpec] = field(default_factory=list)
    kwargs: dict[str, ArgSpec] = field(default_factory=dict)
    require_bbox: bool = False
    allow_extra_kwargs: bool = False
    example: str = ""
    custom_validator: Callable[["CallContext"], None] | None = None


@dataclass
class ToolEntry:
    handler: Callable[["CallContext"], Any]
    spec: MethodSpec


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


def format_method_spec(spec: MethodSpec) -> str:
    lines: list[str] = [f"method: {spec.name}", f"summary: {spec.summary}"]

    if spec.args:
        lines.append("positional args:")
        for idx, arg in enumerate(spec.args):
            req = "required" if arg.required else "optional"
            default = f", default={arg.default}" if arg.default is not None else ""
            lines.append(f"- [{idx}] {arg.name} ({arg.type_text}, {req}{default}): {arg.description}")
    else:
        lines.append("positional args: none")

    if spec.kwargs:
        lines.append("keyword args:")
        for key, arg in spec.kwargs.items():
            req = "required" if arg.required else "optional"
            default = f", default={arg.default}" if arg.default is not None else ""
            lines.append(f"- {key} ({arg.type_text}, {req}{default}): {arg.description}")
    else:
        lines.append("keyword args: none")

    lines.append(f"bbox required: {'yes' if spec.require_bbox else 'no'}")
    if spec.example:
        lines.append(f"example: {spec.example}")
    return "\n".join(lines)


def _validate_type(value: Any, arg_spec: ArgSpec) -> bool:
    if arg_spec.py_types is None:
        return True
    return isinstance(value, arg_spec.py_types)


def validate_call(ctx: CallContext, spec: MethodSpec) -> None:
    if spec.require_bbox and not ctx.bbox:
        raise ToolValidationError("MissingBBox", "this method requires bbox_2d")

    required_pos = sum(1 for a in spec.args if a.required)
    if len(ctx.args) < required_pos:
        raise ToolValidationError(
            "MissingRequiredArgument",
            f"expected at least {required_pos} positional args, got {len(ctx.args)}",
        )
    if len(ctx.args) > len(spec.args):
        raise ToolValidationError(
            "TooManyPositionalArguments",
            f"expected at most {len(spec.args)} positional args, got {len(ctx.args)}",
        )

    for i, val in enumerate(ctx.args):
        arg_spec = spec.args[i]
        if not _validate_type(val, arg_spec):
            raise ToolValidationError(
                "InvalidArgumentType",
                f"arg[{i}] '{arg_spec.name}' expects {arg_spec.type_text}, got {type(val).__name__}",
            )

    if not spec.allow_extra_kwargs:
        unknown = [k for k in ctx.kw_args.keys() if k not in spec.kwargs]
        if unknown:
            raise ToolValidationError("UnknownKeywordArgument", f"unknown keyword args: {unknown}")

    for key, value in ctx.kw_args.items():
        if key not in spec.kwargs:
            continue
        kw_spec = spec.kwargs[key]
        if not _validate_type(value, kw_spec):
            raise ToolValidationError(
                "InvalidArgumentType",
                f"kwarg '{key}' expects {kw_spec.type_text}, got {type(value).__name__}",
            )

    for key, kw_spec in spec.kwargs.items():
        if kw_spec.required and key not in ctx.kw_args:
            raise ToolValidationError("MissingRequiredKeyword", f"missing required keyword arg: {key}")

    if spec.custom_validator:
        spec.custom_validator(ctx)


def build_error_message(ctx: CallContext, exc: Exception, spec: MethodSpec | None = None) -> str:
    label = str(ctx.action.get("label", ""))
    bbox = ctx.action.get("bbox_2d", None)
    base = (
        f"tool call failed: {ctx.func_name}; "
        f"label={label!r}; bbox={bbox}; "
        f"args={ctx.args}; kwargs={ctx.kw_args}; "
        f"error={type(exc).__name__}: {exc}"
    )
    if isinstance(exc, ToolValidationError):
        details = f"\nerror_type: {exc.error_type}\nerror_message: {exc.message}"
        if spec is not None:
            details += "\n" + format_method_spec(spec)
        return base + details
    if spec is not None:
        return base + "\n" + format_method_spec(spec)
    return base


def call(ctx: CallContext, impl: Callable[[CallContext], Any], spec: MethodSpec | None = None) -> CallResult:
    try:
        if spec is not None:
            validate_call(ctx, spec)
        result = impl(ctx)
        if isinstance(result, CallResult):
            return result
        return CallResult(ok=True, value=result)
    except Exception as exc:
        return CallResult(
            ok=False,
            error_message=build_error_message(ctx, exc, spec=spec),
            exception=exc,
        )
