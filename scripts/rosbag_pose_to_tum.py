#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader


def main() -> int:
    ap = argparse.ArgumentParser(description="Extract a PoseStamped topic from rosbag2 into TUM format.")
    ap.add_argument("--bag", required=True, help="rosbag2 directory")
    ap.add_argument("--topic", required=True, help="PoseStamped topic")
    ap.add_argument("--out", required=True, help="Output TUM file")
    args = ap.parse_args()

    bag = Path(args.bag).expanduser().resolve()
    out = Path(args.out).expanduser().resolve()

    rows: list[str] = []
    with AnyReader([bag]) as reader:
        conn = next((c for c in reader.connections if c.topic == args.topic), None)
        if conn is None:
            raise SystemExit(f"topic not found: {args.topic}")

        for c, t, raw in reader.messages(connections=[conn]):
            msg = reader.deserialize(raw, c.msgtype)
            hdr = getattr(msg, "header", None)
            pose = getattr(msg, "pose", None)
            if hdr is None or pose is None:
                continue

            stamp = getattr(hdr, "stamp", None)
            pos = getattr(pose, "position", None)
            ori = getattr(pose, "orientation", None)
            if stamp is None or pos is None or ori is None:
                continue

            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            rows.append(
                f"{ts:.9f} "
                f"{float(pos.x):.9f} {float(pos.y):.9f} {float(pos.z):.9f} "
                f"{float(ori.x):.9f} {float(ori.y):.9f} {float(ori.z):.9f} {float(ori.w):.9f}"
            )

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
