from __future__ import annotations

import os
import time

from .prototype import ArgSpec, CallContext, MethodSpec, ToolValidationError


def _validate_capture(ctx: CallContext) -> None:
    if len(ctx.args) > 2:
        raise ToolValidationError("TooManyPositionalArguments", "capture_evidence accepts at most 2 positional args")

    output_path = ctx.kw_args.get("output_path")
    caption = ctx.kw_args.get("caption")

    if len(ctx.args) >= 1 and output_path is None and caption is None:
        # single positional is treated as caption; 2 positional -> path,caption
        caption = ctx.args[0] if len(ctx.args) == 1 else ctx.args[1]
    elif len(ctx.args) >= 2:
        if output_path is None:
            output_path = ctx.args[0]
        if caption is None:
            caption = ctx.args[1]

    if output_path is not None and not isinstance(output_path, str):
        raise ToolValidationError("InvalidArgumentType", f"output_path expects str, got {type(output_path).__name__}")
    if caption is None or not str(caption).strip():
        raise ToolValidationError("MissingRequiredArgument", "caption is required and must be non-empty")


METHOD_SPEC = MethodSpec(
    name="cature_evidence|capture_evidence",
    summary="Capture screenshot with highlighted bbox and caption text.",
    args=[
        ArgSpec("output_path", "str", "Output image path.", required=False, py_types=(str,)),
        ArgSpec("caption", "str", "Caption text describing evidence.", required=False, py_types=(str,)),
    ],
    kwargs={
        "output_path": ArgSpec("output_path", "str", "Output image path.", required=False, py_types=(str,)),
        "caption": ArgSpec("caption", "str", "Caption text describing evidence.", required=False, py_types=(str,)),
    },
    example='{"bbox_2d": [100,200,500,600], "label": "cature_evidence(output_path=\"logs/evidence.png\", caption=\"这是证据\")"}',
    custom_validator=_validate_capture,
)


def execute(ctx: CallContext) -> None:
    output_path = None
    caption = None

    if "output_path" in ctx.kw_args:
        output_path = str(ctx.kw_args["output_path"])
    if "caption" in ctx.kw_args:
        caption = str(ctx.kw_args["caption"])

    if len(ctx.args) >= 2:
        if output_path is None:
            output_path = str(ctx.args[0])
        if caption is None:
            caption = str(ctx.args[1])
    elif len(ctx.args) == 1 and caption is None:
        caption = str(ctx.args[0])

    if not output_path:
        recorder = getattr(ctx.device, "_trajectory_recorder", None)
        if recorder is not None:
            output_path = os.path.join(recorder.evidence_dir, f"evidence_{int(time.time() * 1000)}.png")
        else:
            output_path = os.path.join(os.getcwd(), "logs", f"evidence_{int(time.time() * 1000)}.png")

    if caption is None or not str(caption).strip():
        raise RuntimeError("Constraint Violation: cature_evidence requires a non-empty caption parameter.")

    dir_path = os.path.dirname(os.path.abspath(output_path))
    if dir_path:
        os.makedirs(dir_path, exist_ok=True)

    img = ctx.device.screenshot().convert("RGB")

    if ctx.bbox and len(ctx.bbox) == 4:
        from PIL import ImageDraw

        x1_raw, y1_raw = ctx.to_pixel(ctx.bbox[0], ctx.bbox[1])
        x2_raw, y2_raw = ctx.to_pixel(ctx.bbox[2], ctx.bbox[3])
        x1, x2 = sorted((x1_raw, x2_raw))
        y1, y2 = sorted((y1_raw, y2_raw))

        x1 = max(0, min(x1, ctx.device.width - 1))
        x2 = max(0, min(x2, ctx.device.width - 1))
        y1 = max(0, min(y1, ctx.device.height - 1))
        y2 = max(0, min(y2, ctx.device.height - 1))

        if x2 > x1 and y2 > y1:
            draw = ImageDraw.Draw(img)
            base_w = max(3, int(min(ctx.device.width, ctx.device.height) * 0.006))
            draw.rectangle([(x1, y1), (x2, y2)], outline=(255, 255, 255), width=base_w + 2)
            draw.rectangle([(x1, y1), (x2, y2)], outline=(255, 0, 0), width=base_w)

    from PIL import Image as PILImage
    from PIL import ImageDraw, ImageFont

    def _load_caption_font(font_size: int):
        candidates = [
            "/usr/share/fonts/wenquanyi/wqy-zenhei/wqy-zenhei.ttc",
            "/usr/share/fonts/wenquanyi/wqy-microhei/wqy-microhei.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKSC-Regular.otf",
            "/usr/share/fonts/truetype/noto/NotoSansCJKSC-Regular.otf",
            "/usr/share/fonts/noto/NotoSansSC-Regular.otf",
            "/usr/share/fonts/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
        for fp in candidates:
            if os.path.exists(fp):
                try:
                    return ImageFont.truetype(fp, font_size)
                except Exception:
                    continue
        return ImageFont.load_default()

    def _text_width(text_draw: ImageDraw.ImageDraw, text: str, font) -> int:
        bbox_text = text_draw.textbbox((0, 0), text, font=font)
        return bbox_text[2] - bbox_text[0]

    def _choose_font_for_width(text_draw: ImageDraw.ImageDraw, max_width: int, target_chars: int = 50):
        min_size = 22
        max_size = 96
        sample_en = ("abcdefghijklmnopqrstuvwxyz" * 2)[:target_chars]

        chosen = _load_caption_font(min_size)
        for size in range(max_size, min_size - 1, -1):
            f = _load_caption_font(size)
            w = _text_width(text_draw, sample_en, f)
            if w <= max_width:
                chosen = f
                break
        return chosen

    def _wrap_text(text_draw: ImageDraw.ImageDraw, text: str, font, max_width: int) -> list[str]:
        lines: list[str] = []
        for paragraph in str(text).splitlines() or [""]:
            if not paragraph:
                lines.append("")
                continue

            units = paragraph.split(" ") if " " in paragraph else list(paragraph)
            sep = " " if " " in paragraph else ""
            current = ""

            for unit in units:
                candidate = (current + sep + unit) if current else unit
                bbox_text = text_draw.textbbox((0, 0), candidate, font=font)
                width = bbox_text[2] - bbox_text[0]
                if width <= max_width:
                    current = candidate
                else:
                    if current:
                        lines.append(current)
                    if sep and text_draw.textbbox((0, 0), unit, font=font)[2] > max_width:
                        token_line = ""
                        for ch in unit:
                            ch_candidate = token_line + ch
                            ch_w = text_draw.textbbox((0, 0), ch_candidate, font=font)[2]
                            if ch_w <= max_width:
                                token_line = ch_candidate
                            else:
                                if token_line:
                                    lines.append(token_line)
                                token_line = ch
                        current = token_line
                    else:
                        current = unit

            if current:
                lines.append(current)

        return lines or [""]

    pad_x = max(20, int(ctx.device.width * 0.03))
    pad_y = max(14, int(ctx.device.height * 0.012))

    measure_draw = ImageDraw.Draw(img)
    max_text_width = max(50, img.width - pad_x * 2)
    font = _choose_font_for_width(measure_draw, max_text_width, target_chars=50)
    caption_lines = _wrap_text(measure_draw, str(caption), font, max_text_width)

    line_bbox = measure_draw.textbbox((0, 0), "Ag测试", font=font)
    line_height = max(30, (line_bbox[3] - line_bbox[1]) + 10)
    caption_h = pad_y * 2 + line_height * len(caption_lines)

    canvas = PILImage.new("RGB", (img.width, img.height + caption_h), color=(255, 255, 255))
    canvas.paste(img, (0, 0))
    canvas_draw = ImageDraw.Draw(canvas)

    divider_y = img.height
    canvas_draw.line([(0, divider_y), (img.width, divider_y)], fill=(220, 220, 220), width=2)

    y = img.height + pad_y
    for line in caption_lines:
        canvas_draw.text((pad_x, y), line, font=font, fill=(20, 20, 20))
        y += line_height

    canvas.save(output_path)
    return output_path
