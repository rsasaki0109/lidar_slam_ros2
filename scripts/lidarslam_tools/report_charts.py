"""Dependency-free SVG and Plotly chart builders for benchmark reports."""

from __future__ import annotations

import html
import json
import math
from typing import Any


LIDAR_COLOR = "#bc4b2f"
GLIM_COLOR = "#24573d"
GRID_COLOR = "rgba(32, 24, 21, 0.12)"


def svg_header(width: int, height: int) -> str:
    return (
        f"<svg viewBox='0 0 {width} {height}' width='100%' height='100%' "
        "xmlns='http://www.w3.org/2000/svg' preserveAspectRatio='xMidYMid meet'>"
    )


def line_chart_svg(
    times: list[float],
    ref_vals: list[float],
    est_vals: list[float],
    title: str,
    y_unit: str,
    width: int = 360,
    height: int = 180,
) -> str:
    if len(times) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 14, 22, 26
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_min = min(times)
    x_max = max(times)
    y_values = ref_vals + est_vals
    y_min = min(y_values)
    y_max = max(y_values)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    pad = (y_max - y_min) * 0.08
    y_min -= pad
    y_max += pad

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    def polyline(timeseries: list[float], values: list[float], color: str) -> str:
        points = " ".join(f"{map_x(tx):.2f},{map_y(v):.2f}" for tx, v in zip(timeseries, values))
        return f"<polyline fill='none' stroke='{color}' stroke-width='2.1' points='{points}' />"

    grid = []
    for idx in range(5):
        frac = idx / 4.0
        y = top + frac * plot_h
        value = y_max - frac * (y_max - y_min)
        grid.append(
            f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />"
            f"<text x='{left - 6}' y='{y + 4:.2f}' text-anchor='end' class='axis'>{value:.2f}</text>"
        )
    grid.append(
        f"<text x='{left + plot_w}' y='{height - 6}' text-anchor='end' class='axis'>{x_max:.1f}s</text>"
    )
    grid.append(
        f"<text x='{left}' y='{height - 6}' text-anchor='start' class='axis'>{x_min:.1f}s</text>"
    )
    zero_line = ""
    if y_min <= 0.0 <= y_max:
        y0 = map_y(0.0)
        zero_line = (
            f"<line x1='{left}' y1='{y0:.2f}' x2='{left + plot_w}' y2='{y0:.2f}' "
            "stroke='rgba(32,24,21,0.24)' stroke-dasharray='4 4' stroke-width='1' />"
        )

    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>{html.escape(title)}</text>"
        + f"<text x='{width - right}' y='14' text-anchor='end' class='axis'>{html.escape(y_unit)}</text>"
        + "".join(grid)
        + zero_line
        + polyline(times, ref_vals, GLIM_COLOR)
        + polyline(times, est_vals, LIDAR_COLOR)
        + "</svg>"
    )


def diff_chart_svg(
    times: list[float],
    vals: list[float],
    title: str,
    y_unit: str,
    width: int = 360,
    height: int = 180,
    color: str = LIDAR_COLOR,
) -> str:
    if len(times) < 2 or len(vals) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 14, 22, 26
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_min = min(times)
    x_max = max(times)
    y_min = min(vals)
    y_max = max(vals)
    if math.isclose(x_min, x_max):
        x_max = x_min + 1.0
    if math.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0
    pad = (y_max - y_min) * 0.08
    y_min -= pad
    y_max += pad

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    points = " ".join(f"{map_x(tx):.2f},{map_y(v):.2f}" for tx, v in zip(times, vals))
    grid = []
    for idx in range(5):
        frac = idx / 4.0
        y = top + frac * plot_h
        value = y_max - frac * (y_max - y_min)
        grid.append(
            f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />"
            f"<text x='{left - 6}' y='{y + 4:.2f}' text-anchor='end' class='axis'>{value:.2f}</text>"
        )
    grid.append(
        f"<text x='{left + plot_w}' y='{height - 6}' text-anchor='end' class='axis'>{x_max:.1f}s</text>"
    )
    grid.append(
        f"<text x='{left}' y='{height - 6}' text-anchor='start' class='axis'>{x_min:.1f}s</text>"
    )
    zero_line = ""
    if y_min <= 0.0 <= y_max:
        y0 = map_y(0.0)
        zero_line = (
            f"<line x1='{left}' y1='{y0:.2f}' x2='{left + plot_w}' y2='{y0:.2f}' "
            "stroke='rgba(32,24,21,0.24)' stroke-dasharray='4 4' stroke-width='1' />"
        )

    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>{html.escape(title)}</text>"
        + f"<text x='{width - right}' y='14' text-anchor='end' class='axis'>{html.escape(y_unit)}</text>"
        + "".join(grid)
        + zero_line
        + f"<polyline fill='none' stroke='{color}' stroke-width='2.1' points='{points}' />"
        + "</svg>"
    )


def xy_chart_svg(
    ref_x: list[float],
    ref_y: list[float],
    est_x: list[float],
    est_y: list[float],
    width: int = 720,
    height: int = 420,
) -> str:
    if len(ref_x) < 2 or len(est_x) < 2:
        return "<div class='plot-empty'>not enough points</div>"

    left, right, top, bottom = 42, 20, 20, 32
    plot_w = width - left - right
    plot_h = height - top - bottom
    x_all = ref_x + est_x
    y_all = ref_y + est_y
    x_min = min(x_all)
    x_max = max(x_all)
    y_min = min(y_all)
    y_max = max(y_all)
    span = max(x_max - x_min, y_max - y_min, 1e-6)
    x_mid = (x_min + x_max) * 0.5
    y_mid = (y_min + y_max) * 0.5
    x_min = x_mid - span * 0.55
    x_max = x_mid + span * 0.55
    y_min = y_mid - span * 0.55
    y_max = y_mid + span * 0.55

    def map_x(value: float) -> float:
        return left + ((value - x_min) / (x_max - x_min)) * plot_w

    def map_y(value: float) -> float:
        return top + plot_h - ((value - y_min) / (y_max - y_min)) * plot_h

    def polyline(xs: list[float], ys: list[float], color: str) -> str:
        points = " ".join(f"{map_x(px):.2f},{map_y(py):.2f}" for px, py in zip(xs, ys))
        return f"<polyline fill='none' stroke='{color}' stroke-width='2.2' points='{points}' />"

    grid = []
    for idx in range(5):
        frac = idx / 4.0
        x = left + frac * plot_w
        y = top + frac * plot_h
        grid.append(f"<line x1='{x:.2f}' y1='{top}' x2='{x:.2f}' y2='{top + plot_h}' stroke='{GRID_COLOR}' stroke-width='1' />")
        grid.append(f"<line x1='{left}' y1='{y:.2f}' x2='{left + plot_w}' y2='{y:.2f}' stroke='{GRID_COLOR}' stroke-width='1' />")
    start_ref = f"<circle cx='{map_x(ref_x[0]):.2f}' cy='{map_y(ref_y[0]):.2f}' r='4.5' fill='{GLIM_COLOR}' />"
    start_est = f"<circle cx='{map_x(est_x[0]):.2f}' cy='{map_y(est_y[0]):.2f}' r='4.5' fill='{LIDAR_COLOR}' />"
    return (
        svg_header(width, height)
        + f"<text x='{left}' y='14' class='plot-title'>XY Trajectory Overlay</text>"
        + f"<text x='{left}' y='{height - 8}' class='axis'>x [{x_min:.2f}, {x_max:.2f}] m</text>"
        + f"<text x='{width - right}' y='{height - 8}' text-anchor='end' class='axis'>y [{y_min:.2f}, {y_max:.2f}] m</text>"
        + "".join(grid)
        + polyline(ref_x, ref_y, GLIM_COLOR)
        + polyline(est_x, est_y, LIDAR_COLOR)
        + start_ref
        + start_est
        + "</svg>"
    )


def plotly_3d_chart(
    series: dict[str, Any],
    plot_id: str,
) -> str:
    data = [
        {
            "type": "scatter3d",
            "mode": "lines",
            "name": "GLIM",
            "x": series["ref_xyz"]["x"],
            "y": series["ref_xyz"]["y"],
            "z": series["ref_xyz"]["z"],
            "line": {"color": GLIM_COLOR, "width": 4},
        },
        {
            "type": "scatter3d",
            "mode": "lines",
            "name": "lidarslam",
            "x": series["est_xyz"]["x"],
            "y": series["est_xyz"]["y"],
            "z": series["est_xyz"]["z"],
            "line": {"color": LIDAR_COLOR, "width": 4},
        },
    ]
    layout = {
        "margin": {"l": 0, "r": 0, "t": 34, "b": 0},
        "paper_bgcolor": "rgba(255,255,255,0)",
        "plot_bgcolor": "rgba(255,255,255,0)",
        "legend": {
            "orientation": "h",
            "x": 0.0,
            "y": 1.08,
            "bgcolor": "rgba(255,255,255,0.0)",
        },
        "scene": {
            "xaxis": {"title": "X [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "yaxis": {"title": "Y [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "zaxis": {"title": "Z [m]", "gridcolor": GRID_COLOR, "zerolinecolor": GRID_COLOR},
            "aspectmode": "data",
            "dragmode": "orbit",
        },
    }
    config = {"responsive": True, "displaylogo": False, "scrollZoom": True}
    div_id = f"{plot_id}-3d"
    return (
        f"<div class='plot-card plotly-card'><div id='{div_id}' class='plotly-3d'></div></div>"
        f"<script>Plotly.newPlot({json.dumps(div_id)}, {json.dumps(data)}, {json.dumps(layout)}, {json.dumps(config)});</script>"
    )


