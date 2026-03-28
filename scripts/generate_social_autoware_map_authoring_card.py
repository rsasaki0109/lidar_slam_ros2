#!/usr/bin/env python3
"""Generate a social PNG promoting the Autoware-compatible map-authoring path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


ROOT = Path(__file__).resolve().parents[1]
VERSION_PATH = ROOT / "VERSION"
LOOP_IMAGE = ROOT / "lidarslam" / "images" / "mid360_loop_closure_zoom.png"
NTU_METRICS = (
    ROOT / "output" / "bench_rko_lio_ntu_viral_fresh_20260324" / "metrics.json"
)
MID360_METRICS = (
    ROOT / "output" / "bench_rko_lio_mid360_current_default_20260325" / "metrics.json"
)
DYNAMIC_FILTER_REPORT = (
    ROOT
    / "output"
    / "dynamic_object_filter_benchmark_bag6_20260326"
    / "dynamic_object_filter_report.json"
)
DEFAULT_OUT = ROOT / "lidarslam" / "images" / "social_autoware_map_authoring.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--loop-image", default=str(LOOP_IMAGE))
    return parser.parse_args()


def _load_font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = []
    if bold:
        candidates.extend(
            [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
            ]
        )
    candidates.extend(
        [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        ]
    )
    for candidate in candidates:
        path = Path(candidate)
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def _wrap(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont, width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = word if not current else f"{current} {word}"
        if draw.textlength(candidate, font=font) <= width:
            current = candidate
            continue
        if current:
            lines.append(current)
        current = word
    if current:
        lines.append(current)
    return lines


def _draw_wrapped(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    font: ImageFont.ImageFont,
    fill: str,
    width: int,
    *,
    line_gap: int = 6,
) -> int:
    x, y = xy
    line_height = font.size + line_gap
    for line in _wrap(draw, text, font, width):
        draw.text((x, y), line, font=font, fill=fill)
        y += line_height
    return y


def _round_image(image: Image.Image, radius: int) -> Image.Image:
    image = image.convert("RGBA")
    mask = Image.new("L", image.size, 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.rounded_rectangle((0, 0, image.width, image.height), radius=radius, fill=255)
    image.putalpha(mask)
    return image


def _metric(path: Path, key_path: Iterable[str], fallback: str) -> str:
    if not path.is_file():
        return fallback
    payload = json.loads(path.read_text(encoding="utf-8"))
    current: object = payload
    for key in key_path:
        if not isinstance(current, dict) or key not in current:
            return fallback
        current = current[key]
    if isinstance(current, float):
        return f"{current:.3f}"
    return str(current)


def main() -> None:
    args = parse_args()
    out_path = Path(args.out).expanduser().resolve()
    loop_image_path = Path(args.loop_image).expanduser().resolve()

    version = VERSION_PATH.read_text(encoding="utf-8").strip()
    ntu_rmse = _metric(NTU_METRICS, ["evo", "ape", "rmse"], "n/a")
    mid360_rmse = _metric(MID360_METRICS, ["evo", "ape", "rmse"], "n/a")
    reduction_ratio = _metric(DYNAMIC_FILTER_REPORT, ["point_reduction_ratio"], "n/a")
    if reduction_ratio != "n/a":
        reduction_ratio = f"{float(reduction_ratio) * 100.0:.1f}%"

    canvas = Image.new("RGB", (1600, 900), "#f3f6fb")
    draw = ImageDraw.Draw(canvas)
    title_font = _load_font(56, bold=True)
    badge_font = _load_font(22, bold=True)
    body_font = _load_font(28)
    body_bold_font = _load_font(28, bold=True)
    metric_label_font = _load_font(18)
    metric_value_font = _load_font(34, bold=True)
    small_font = _load_font(20)

    draw.rounded_rectangle((36, 36, 1564, 864), radius=32, fill="#ffffff", outline="#d6e1ef")
    draw.rounded_rectangle((64, 64, 388, 108), radius=18, fill="#0f172a")
    draw.text((86, 77), f"v{version}  |  non-GPL path", font=badge_font, fill="#f8fafc")

    left_x = 88
    y = 138
    y = _draw_wrapped(
        draw,
        (left_x, y),
        "Autoware-compatible pointcloud-map authoring",
        title_font,
        "#0f172a",
        650,
        line_gap=10,
    )
    y += 14
    y = _draw_wrapped(
        draw,
        (left_x, y),
        "ROS 2 workflow for reproducible map output, GNSS metadata, release-ready verification, and tracked benchmark artifacts.",
        body_font,
        "#334155",
        640,
    )

    bullets = [
        "RKO-LIO + graph_based_slam public path",
        "Outputs pointcloud_map/ and map_projector_info.yaml",
    ]
    y += 26
    for bullet in bullets:
        draw.ellipse((left_x, y + 10, left_x + 10, y + 20), fill="#2563eb")
        y = _draw_wrapped(draw, (left_x + 24, y), bullet, body_font, "#0f172a", 610)
        y += 10

    draw.rounded_rectangle((80, 580, 690, 684), radius=20, fill="#eff6ff", outline="#bfdbfe")
    draw.text((108, 602), "Quickstart", font=body_bold_font, fill="#1d4ed8")
    draw.text((108, 640), "bash scripts/run_autoware_quickstart.sh", font=_load_font(24), fill="#0f172a")

    metric_cards = [
        ("NTU VIRAL current default", f"{ntu_rmse} m"),
        ("MID360 current default", f"{mid360_rmse} m"),
        ("Leo Drive bag6 point reduction", reduction_ratio),
    ]
    card_y = 710
    card_x = 80
    card_w = 176
    card_h = 116
    card_gap = 16
    for label, value in metric_cards:
        draw.rounded_rectangle(
            (card_x, card_y, card_x + card_w, card_y + card_h),
            radius=18,
            fill="#f8fafc",
            outline="#d6e1ef",
        )
        _draw_wrapped(
            draw,
            (card_x + 16, card_y + 14),
            label,
            metric_label_font,
            "#475569",
            card_w - 28,
            line_gap=1,
        )
        draw.text((card_x + 16, card_y + 74), value, font=metric_value_font, fill="#0f172a")
        card_x += card_w + card_gap

    if loop_image_path.is_file():
        loop_image = Image.open(loop_image_path).convert("RGB")
        right_image = loop_image.resize((700, 540))
        right_image = _round_image(right_image, 28)
        canvas.paste(right_image, (828, 112), right_image)
        draw.rounded_rectangle((828, 112, 1528, 652), radius=28, outline="#d6e1ef", width=2)

    draw.rounded_rectangle((828, 684, 1528, 820), radius=24, fill="#0f172a")
    draw.text((860, 714), "What the card is showing", font=body_bold_font, fill="#f8fafc")
    footer = (
        "Current MID360 loop-area zoom on the right. The left side summarizes the "
        "public map-authoring path, tracked benchmark numbers, and save-time cleanup evidence."
    )
    _draw_wrapped(draw, (860, 754), footer, small_font, "#cbd5e1", 636, line_gap=4)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
    print(out_path)


if __name__ == "__main__":
    main()
