#!/usr/bin/env python3
"""Generate a short social demo video for Autoware-compatible map authoring."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[1]
IMAGES_DIR = REPO_ROOT / 'lidarslam' / 'images'
DEFAULT_OUTPUT = IMAGES_DIR / 'social_autoware_map_authoring_demo.mp4'
SIZE = (1280, 720)
FPS = 24
SLIDE_SECONDS = 3.0
FADE_SECONDS = 0.45


def _load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = []
    if bold:
        candidates.extend([
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        ])
    else:
        candidates.extend([
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        ])
    for candidate in candidates:
        path = Path(candidate)
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def _fit_image(path: Path, size: tuple[int, int]) -> Image.Image:
    canvas = Image.new('RGB', size, '#0b1220')
    source = Image.open(path).convert('RGB')
    scale = min(size[0] / source.width, size[1] / source.height)
    scaled = source.resize(
        (max(1, int(source.width * scale)), max(1, int(source.height * scale))),
        Image.Resampling.LANCZOS,
    )
    offset = ((size[0] - scaled.width) // 2, (size[1] - scaled.height) // 2)
    canvas.paste(scaled, offset)
    return canvas


def _draw_wrapped_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.ImageFont,
    fill: str,
    xy: tuple[int, int],
    max_width: int,
    line_spacing: int,
) -> int:
    words = text.split()
    lines: list[str] = []
    current = ''
    for word in words:
        candidate = word if not current else f'{current} {word}'
        bbox = draw.textbbox((0, 0), candidate, font=font)
        if bbox[2] - bbox[0] <= max_width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)

    x, y = xy
    for line in lines:
        draw.text((x, y), line, font=font, fill=fill)
        bbox = draw.textbbox((x, y), line, font=font)
        y = bbox[3] + line_spacing
    return y


def _overlay_card(canvas: Image.Image, title: str, body: list[str], eyebrow: str | None = None) -> Image.Image:
    overlay = Image.new('RGBA', SIZE, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    panel = (56, 56, SIZE[0] - 56, SIZE[1] - 56)
    draw.rounded_rectangle(panel, radius=28, fill=(7, 12, 21, 196))

    title_font = _load_font(46, bold=True)
    body_font = _load_font(26)
    eyebrow_font = _load_font(20, bold=True)

    cursor_y = 92
    if eyebrow:
        draw.text((88, cursor_y), eyebrow.upper(), font=eyebrow_font, fill='#8dd3ff')
        cursor_y += 42
    draw.text((88, cursor_y), title, font=title_font, fill='white')
    cursor_y += 78

    for item in body:
        cursor_y = _draw_wrapped_text(
            draw,
            f'- {item}',
            body_font,
            '#d8e4f0',
            (96, cursor_y),
            SIZE[0] - 192,
            14,
        )
        cursor_y += 8

    combined = Image.alpha_composite(canvas.convert('RGBA'), overlay)
    return combined.convert('RGB')


def _make_slide_title() -> Image.Image:
    canvas = _fit_image(IMAGES_DIR / 'social_autoware_map_authoring.png', SIZE)
    return _overlay_card(
        canvas,
        'Autoware-Compatible Map Authoring',
        [
            'ROS 2 pointcloud-map workflow with a beginner-friendly one-command entrypoint.',
            'Build a map bundle, verify it, and open it through Autoware-compatible viewers.',
        ],
        eyebrow='lidarslam_ros2 v0.2.2',
    )


def _make_slide_proof() -> Image.Image:
    canvas = _fit_image(IMAGES_DIR / 'autoware_map_loader_proof.png', SIZE)
    return _overlay_card(
        canvas,
        'Live Browser Proof',
        [
            'Autoware map loaders publish /map/pointcloud_map.',
            'The documented public path keeps verify_autoware_map.py in the loop.',
            'GNSS-enabled runs write LocalCartesian map metadata.',
        ],
        eyebrow='foxglove + autoware',
    )


def _make_slide_loop() -> Image.Image:
    canvas = _fit_image(IMAGES_DIR / 'mid360_loop_closure_zoom.png', SIZE)
    return _overlay_card(
        canvas,
        'Tracked Mapping Evidence',
        [
            'NTU VIRAL current default: APE RMSE 0.952 m',
            'MID360 current default: APE RMSE 3.641 m',
            'Leo Drive dynamic filter: about 50% saved-point reduction',
        ],
        eyebrow='benchmarks',
    )


def _make_slide_cta() -> Image.Image:
    canvas = Image.new('RGB', SIZE, '#08101b')
    return _overlay_card(
        canvas,
        'Start With One Command',
        [
            'bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2',
            'Docs: rsasaki0109.github.io/lidarslam_ros2/',
            'Release: github.com/rsasaki0109/lidarslam_ros2/releases/tag/v0.2.2',
        ],
        eyebrow='quickstart',
    )


def _write_slides(temp_dir: Path) -> list[Path]:
    builders = [
        _make_slide_title,
        _make_slide_proof,
        _make_slide_loop,
        _make_slide_cta,
    ]
    paths = []
    for index, builder in enumerate(builders, start=1):
        slide = builder()
        path = temp_dir / f'slide_{index:02d}.png'
        slide.save(path, format='PNG')
        paths.append(path)
    return paths


def _ffmpeg_cmd(slides: list[Path], output: Path) -> list[str]:
    cmd = ['ffmpeg', '-y']
    for slide in slides:
        cmd.extend([
            '-loop', '1',
            '-t', str(SLIDE_SECONDS),
            '-i', str(slide),
        ])

    chains = []
    for index in range(len(slides)):
        chains.append(
            f'[{index}:v]fps={FPS},scale={SIZE[0]}:{SIZE[1]}:force_original_aspect_ratio=decrease,'
            f'pad={SIZE[0]}:{SIZE[1]}:(ow-iw)/2:(oh-ih)/2,setsar=1[v{index}]'
        )

    current = 'v0'
    offset = SLIDE_SECONDS - FADE_SECONDS
    for index in range(1, len(slides)):
        next_name = f'v{index}'
        out_name = f'x{index}'
        chains.append(
            f'[{current}][{next_name}]xfade=transition=fade:duration={FADE_SECONDS}:offset={offset}[{out_name}]'
        )
        current = out_name
        offset += SLIDE_SECONDS - FADE_SECONDS
    chains.append(f'[{current}]format=yuv420p[video]')

    cmd.extend([
        '-filter_complex', ';'.join(chains),
        '-map', '[video]',
        '-r', str(FPS),
        '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart',
        '-crf', '23',
        str(output),
    ])
    return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Generate a short social demo video.')
    parser.add_argument(
        '--output',
        default=str(DEFAULT_OUTPUT),
        help='Output MP4 path.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    if shutil.which('ffmpeg') is None:
        raise SystemExit('ffmpeg is required but was not found on PATH')

    with tempfile.TemporaryDirectory(prefix='autoware_demo_video_') as temp_dir_str:
        temp_dir = Path(temp_dir_str)
        slides = _write_slides(temp_dir)
        subprocess.run(_ffmpeg_cmd(slides, output), check=True)

    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
