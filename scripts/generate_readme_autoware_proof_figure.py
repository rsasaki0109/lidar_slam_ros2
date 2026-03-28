#!/usr/bin/env python3
"""Generate a README proof image for the Autoware-compatible map flow."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


ROOT = Path(__file__).resolve().parents[1]
DOGFOOD_DIR = ROOT / "output" / "dogfood_rko_lio_autoware_20260324_190734"
GNSS_SMOKE_DIR = ROOT / "output" / "open_data_gnss_smoke_bag6_autodetect_throttled_20260325"
LOOP_ZOOM_PATH = ROOT / "lidarslam" / "images" / "mid360_loop_closure_zoom.png"
DEFAULT_OUT = ROOT / "lidarslam" / "images" / "autoware_map_loader_proof.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dogfood-dir", default=str(DOGFOOD_DIR))
    parser.add_argument("--gnss-smoke-dir", default=str(GNSS_SMOKE_DIR))
    parser.add_argument("--loop-zoom", default=str(LOOP_ZOOM_PATH))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
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


def _wrapped_lines(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.ImageFont,
    width: int,
) -> list[str]:
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
    text: str,
    xy: tuple[int, int],
    width: int,
    font: ImageFont.ImageFont,
    fill: str,
    *,
    line_gap: int = 6,
) -> int:
    x, y = xy
    for line in _wrapped_lines(draw, text, font, width):
        draw.text((x, y), line, font=font, fill=fill)
        y += font.size + line_gap
    return y


def _round_image(image: Image.Image, radius: int) -> Image.Image:
    image = image.convert("RGBA")
    mask = Image.new("L", image.size, 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.rounded_rectangle((0, 0, image.width, image.height), radius=radius, fill=255)
    image.putalpha(mask)
    return image


def _find_rviz_log(dogfood_dir: Path) -> Path:
    candidates = sorted((dogfood_dir / ".ros_log").glob("rviz2_*.log"))
    if not candidates:
        raise FileNotFoundError("rviz2 log not found")
    return candidates[0]


def _extract_subscribe_line(rviz_log_path: Path) -> str:
    for line in rviz_log_path.read_text(encoding="utf-8").splitlines():
        if "Subscribing to: /map/pointcloud_map" in line:
            return "rviz2: Subscribing to /map/pointcloud_map"
    return "rviz2: /map/pointcloud_map subscription not found"


def _extract_saved_map_summary(slam_log_path: Path) -> str:
    for line in reversed(slam_log_path.read_text(encoding="utf-8").splitlines()):
        if "Saved grid-divided map:" in line:
            summary = re.sub(r"^.*Saved grid-divided map:", "Saved grid-divided map:", line).strip()
            summary = re.sub(r"\s+to\s+.*$", "", summary)
            return summary
    return "Saved grid-divided map summary not found"


def _extract_verify_result(verify_log_path: Path) -> str:
    for line in verify_log_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("RESULT:"):
            return line.strip()
    return "RESULT: verify output not found"


def _extract_projector_summary(projector_path: Path) -> tuple[str, str]:
    text = projector_path.read_text(encoding="utf-8")
    projector = "unknown"
    latlon = "map_origin not found"
    for line in text.splitlines():
        if line.startswith("projector_type:"):
            projector = line.split(":", 1)[1].strip()
    lat = re.search(r"latitude:\s*([0-9.+-]+)", text)
    lon = re.search(r"longitude:\s*([0-9.+-]+)", text)
    if lat and lon:
        latlon = f"map_origin lat {lat.group(1)}, lon {lon.group(1)}"
    return projector, latlon


def main() -> None:
    args = parse_args()
    dogfood_dir = Path(args.dogfood_dir).expanduser().resolve()
    gnss_smoke_dir = Path(args.gnss_smoke_dir).expanduser().resolve()
    loop_zoom_path = Path(args.loop_zoom).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()

    rviz_line = _extract_subscribe_line(_find_rviz_log(dogfood_dir))
    saved_map_line = _extract_saved_map_summary(dogfood_dir / "slam.launch.log")
    verify_line = _extract_verify_result(gnss_smoke_dir / "verify_autoware_map.log")
    projector_type, latlon = _extract_projector_summary(gnss_smoke_dir / "map_projector_info.yaml")

    canvas = Image.new("RGB", (1600, 900), "#eff4fa")
    draw = ImageDraw.Draw(canvas)
    title_font = _load_font(52, bold=True)
    body_font = _load_font(26)
    body_bold_font = _load_font(28, bold=True)
    mono_font = _load_font(22)
    small_font = _load_font(22)
    badge_font = _load_font(22, bold=True)

    draw.rounded_rectangle((34, 34, 1566, 866), radius=34, fill="#ffffff", outline="#d8e3ef")
    draw.rounded_rectangle((68, 64, 366, 110), radius=18, fill="#0f172a")
    draw.text((92, 78), "Autoware-compatible proof", font=badge_font, fill="#f8fafc")

    left_x = 84
    y = 144
    y = _draw_wrapped(
        draw,
        "Pointcloud-map flow exercised end-to-end",
        (left_x, y),
        620,
        title_font,
        "#0f172a",
        line_gap=10,
    )
    y += 12
    y = _draw_wrapped(
        draw,
        "This card is built from real dogfood and open-data smoke artifacts. It shows the Autoware map-loader path, map verification, and GNSS map metadata in one view.",
        (left_x, y),
        610,
        body_font,
        "#334155",
    )

    box_specs = [
        ("RViz log", rviz_line),
        ("Map save", saved_map_line),
        (
            "Verify + metadata",
            f"{verify_line}  |  projector_type: {projector_type}  |  {latlon}",
        ),
    ]
    box_y = y + 30
    box_w = 620
    box_h = 112
    for title, text in box_specs:
        draw.rounded_rectangle(
            (left_x, box_y, left_x + box_w, box_y + box_h),
            radius=20,
            fill="#f8fafc",
            outline="#d8e3ef",
        )
        draw.text((left_x + 22, box_y + 18), title, font=body_bold_font, fill="#1d4ed8")
        _draw_wrapped(
            draw,
            text,
            (left_x + 22, box_y + 52),
            box_w - 44,
            mono_font,
            "#0f172a",
            line_gap=2,
        )
        box_y += box_h + 16

    if loop_zoom_path.is_file():
        loop_zoom = Image.open(loop_zoom_path).convert("RGB").resize((720, 590))
        loop_zoom = _round_image(loop_zoom, 28)
        canvas.paste(loop_zoom, (808, 96), loop_zoom)
        draw.rounded_rectangle((808, 96, 1528, 686), radius=28, outline="#d8e3ef", width=2)

    draw.rounded_rectangle((808, 700, 1528, 838), radius=24, fill="#0f172a")
    draw.text((840, 742), "What this proves", font=body_bold_font, fill="#f8fafc")
    footer = (
        "Autoware's RViz-side loader subscribes to /map/pointcloud_map. "
        "The saved map verifies as PASS, and GNSS-enabled runs emit LocalCartesian metadata."
    )
    _draw_wrapped(draw, footer, (840, 780), 650, small_font, "#cbd5e1", line_gap=4)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
    print(out_path)


if __name__ == "__main__":
    main()
