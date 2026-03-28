#!/usr/bin/env python3
"""Generate a browser-based proof image from a live /map/pointcloud_map."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parents[1]
AUTOWARE_MAP_DIR = ROOT / "output" / "open_data_gnss_smoke_bag6_autodetect_throttled_20260325"
DEFAULT_OUT = ROOT / "lidarslam" / "images" / "autoware_map_loader_proof.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--autoware-map-dir", default=str(AUTOWARE_MAP_DIR))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--max-points", type=int, default=50000)
    return parser.parse_args()


def _extract_verify_result(verify_log_path: Path) -> str:
    for line in verify_log_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("RESULT:"):
            return line.strip()
    return "RESULT: unknown"


def _extract_projector_summary(projector_path: Path) -> tuple[str, str]:
    projector_type = "unknown"
    map_origin = "map_origin: unavailable"
    for raw in projector_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("projector_type:"):
            projector_type = line.split(":", 1)[1].strip()
    text = projector_path.read_text(encoding="utf-8")
    lat = None
    lon = None
    for raw in text.splitlines():
        stripped = raw.strip()
        if stripped.startswith("latitude:"):
            lat = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("longitude:"):
            lon = stripped.split(":", 1)[1].strip()
    if lat and lon:
        map_origin = f"map_origin: lat {lat}, lon {lon}"
    return projector_type, map_origin


def _capture_pointcloud(max_points: int) -> tuple[np.ndarray, dict[str, object]]:
    capture_code = f"""
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

out_path = Path(r"{tempfile.gettempdir()}") / "autoware_pointcloud_capture.json"

class CaptureNode(Node):
    def __init__(self):
        super().__init__("autoware_pointcloud_capture")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cloud = None
        self.create_subscription(PointCloud2, "/map/pointcloud_map", self.on_cloud, qos)

    def on_cloud(self, msg):
        self.cloud = msg

rclpy.init()
node = CaptureNode()
deadline = time.time() + 40.0
while time.time() < deadline and node.cloud is None:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.cloud is None:
    raise SystemExit("did not receive /map/pointcloud_map")

raw_points = point_cloud2.read_points(node.cloud, field_names=("x", "y", "z"), skip_nans=True)
points = np.column_stack((raw_points["x"], raw_points["y"], raw_points["z"])).astype(np.float32, copy=False)
if points.size == 0:
    raise SystemExit("received empty point cloud")

if len(points) > {max_points}:
    step = max(1, len(points) // {max_points})
    points = points[::step][: {max_points}]

out = {{
    "width": int(node.cloud.width),
    "height": int(node.cloud.height),
    "frame_id": node.cloud.header.frame_id,
    "points": points.tolist(),
}}
out_path.write_text(json.dumps(out), encoding="utf-8")
print(out_path)

node.destroy_node()
rclpy.shutdown()
"""
    env = os.environ.copy()
    proc = subprocess.run(
        ["bash", "-lc", f"set -eo pipefail; source /opt/ros/jazzy/setup.bash; python3 - <<'PY'\n{capture_code}\nPY"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        env=env,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "failed to capture /map/pointcloud_map\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )
    capture_path = Path(proc.stdout.strip().splitlines()[-1])
    payload = json.loads(capture_path.read_text(encoding="utf-8"))
    points = np.asarray(payload["points"], dtype=np.float32)
    return points, payload


def _build_html(
    points: np.ndarray,
    payload: dict[str, object],
    verify_result: str,
    projector_type: str,
    map_origin: str,
) -> str:
    z_values = points[:, 2]
    fig = go.Figure(
        data=[
            go.Scattergl(
                x=points[:, 0],
                y=points[:, 1],
                mode="markers",
                marker={
                    "size": 2,
                    "opacity": 0.72,
                    "color": z_values,
                    "colorscale": "Turbo",
                    "colorbar": {"title": "z (m)", "x": 1.02},
                },
                hoverinfo="skip",
            )
        ]
    )
    fig.update_layout(
        width=1600,
        height=900,
        margin={"l": 30, "r": 80, "t": 110, "b": 40},
        paper_bgcolor="#0f172a",
        plot_bgcolor="#0f172a",
        xaxis={"title": "x (m)", "gridcolor": "#1e293b", "zerolinecolor": "#334155", "color": "#e2e8f0"},
        yaxis={
            "title": "y (m)",
            "scaleanchor": "x",
            "scaleratio": 1,
            "gridcolor": "#1e293b",
            "zerolinecolor": "#334155",
            "color": "#e2e8f0",
        },
        font={"color": "#e2e8f0"},
        title={
            "text": "Autoware-compatible /map/pointcloud_map proof",
            "font": {"size": 30},
            "x": 0.02,
            "y": 0.98,
            "xanchor": "left",
        },
    )
    plot_html = fig.to_html(include_plotlyjs=True, full_html=False, div_id="plot")
    summary_lines = [
        "/map/pointcloud_map received from Autoware map loader",
        f"frame_id: {payload['frame_id']}",
        f"cloud: {payload['width']} x {payload['height']} samples shown: {len(points)}",
        f"projector_type: {projector_type}",
        map_origin,
        verify_result,
    ]
    summary_html = "".join(f"<li>{line}</li>" for line in summary_lines)
    return f"""<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <title>Autoware Map Loader Proof</title>
    <style>
      body {{
        margin: 0;
        background: #020617;
        font-family: 'DejaVu Sans', sans-serif;
      }}
      .frame {{
        width: 1600px;
        height: 900px;
        position: relative;
        overflow: hidden;
      }}
      .overlay {{
        position: absolute;
        top: 18px;
        left: 22px;
        z-index: 10;
        max-width: 510px;
        background: rgba(15, 23, 42, 0.88);
        border: 1px solid rgba(148, 163, 184, 0.45);
        border-radius: 18px;
        padding: 16px 18px;
        color: #f8fafc;
        box-shadow: 0 18px 48px rgba(2, 6, 23, 0.45);
      }}
      .badge {{
        display: inline-block;
        margin-bottom: 10px;
        padding: 6px 10px;
        border-radius: 999px;
        background: #16a34a;
        color: #f8fafc;
        font-size: 14px;
        font-weight: 700;
      }}
      h1 {{
        margin: 0 0 8px 0;
        font-size: 28px;
        line-height: 1.15;
      }}
      p {{
        margin: 0 0 10px 0;
        font-size: 17px;
        color: #cbd5e1;
      }}
      ul {{
        margin: 0;
        padding-left: 20px;
        font-size: 16px;
        line-height: 1.5;
      }}
      li {{
        margin-bottom: 4px;
      }}
      #plot {{
        width: 1600px;
        height: 900px;
      }}
    </style>
  </head>
  <body>
    <div class="frame">
      {plot_html}
      <div class="overlay">
        <div class="badge">Browser Proof</div>
        <h1>Autoware map loader publishes the saved pointcloud map</h1>
        <p>Actual browser rendering from a live <code>/map/pointcloud_map</code> message.</p>
        <ul>{summary_html}</ul>
      </div>
    </div>
  </body>
</html>
"""


def _render_html_to_png(html_path: Path, out_path: Path) -> None:
    with sync_playwright() as p:
        browser = p.chromium.launch(channel="chrome", headless=True)
        page = browser.new_page(viewport={"width": 1600, "height": 900, "device_scale_factor": 1})
        page.goto(html_path.as_uri(), wait_until="load", timeout=60000)
        page.wait_for_timeout(3000)
        page.screenshot(path=str(out_path), full_page=False)
        browser.close()


def main() -> None:
    args = parse_args()
    autoware_map_dir = Path(args.autoware_map_dir).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()

    verify_result = _extract_verify_result(autoware_map_dir / "verify_autoware_map.log")
    projector_type, map_origin = _extract_projector_summary(autoware_map_dir / "map_projector_info.yaml")
    points, payload = _capture_pointcloud(args.max_points)
    html = _build_html(points, payload, verify_result, projector_type, map_origin)
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False, encoding="utf-8") as handle:
        handle.write(html)
        html_path = Path(handle.name)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    _render_html_to_png(html_path, out_path)


if __name__ == "__main__":
    main()
