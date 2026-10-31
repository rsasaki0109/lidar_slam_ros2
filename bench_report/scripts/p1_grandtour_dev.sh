#!/usr/bin/env bash
# p1_grandtour_dev.sh <label> <hf_folder:short> ... (chapter 31): run p1_grandtour_run.sh (G_CONFIG / IMU passed through)
# on GrandTour development missions (data/grandtour/dev_missions_2026-10-27.txt, HEAP-1, ARC-3), then, after the
# trajectory is written, fetch the mission's prism reference and append label,short,<p1_grandtour_ate.py line> to
# /home/user/bench/grandtour/dev_scores.csv. The bag is kept for further variants; the extracted point arrays are
# deleted (only the zarr metadata, which hold the extrinsics, are kept).
set -euo pipefail
label=$1; shift
S=/home/user/lidar_slam_ros2/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
HF=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main
for item in "$@"; do
  folder=${item%%:*}; short=${item##*:}
  bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${WS:-/home/user/ws/rko_g12}"
  # two variants on one bag may finish together: a file the other run already deleted is not an error
  find "/home/user/data/grandtour/${short}/z" -type f ! -name '.z*' -delete 2>/dev/null || true
  ref=/home/user/data/grandtour/reference_dev/${short}
  if [[ ! -d ${ref}/prism_position ]]; then
    mkdir -p "${ref}"
    curl -sS -L --fail -o "${ref}/prism_position.tar" "${HF}/${folder}/data/prism_position.tar"
    tar xf "${ref}/prism_position.tar" -C "${ref}"; rm -f "${ref}/prism_position.tar"
  fi
  line=$("${PY}" -I "${S}/p1_grandtour_ate.py" "/home/user/bench/grandtour/${label}/${short}/${short}.tum" "${ref}/prism_position")
  echo "${label},${short},${line}" | tee -a /home/user/bench/grandtour/dev_scores.csv
done
