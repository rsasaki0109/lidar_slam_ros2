#!/usr/bin/env bash
set -euo pipefail

echo "=== AWSIM + Autoware セットアップ確認 ==="
echo ""

AUTOWARE_IMAGE="${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-cuda}"

# 1. DDS カーネルパラメータ設定
echo "[1/5] CycloneDDS カーネルパラメータ設定..."
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo ip link set lo multicast on
echo "  OK"

# 2. CycloneDDS XML 確認
echo "[2/5] CycloneDDS XML..."
if [ -f "${HOME}/cyclonedds.xml" ]; then
  echo "  OK: ${HOME}/cyclonedds.xml"
else
  echo "  FAIL: ${HOME}/cyclonedds.xml が見つかりません"
  exit 1
fi

# 3. Docker + GPU 確認
echo "[3/5] Docker GPU パススルー..."
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi --query-gpu=name --format=csv,noheader
echo "  OK"

# 4. Autoware Docker イメージ確認
echo "[4/5] Autoware Docker イメージ..."
if docker image inspect "${AUTOWARE_IMAGE}" >/dev/null 2>&1; then
  SIZE=$(docker images "${AUTOWARE_IMAGE}" --format '{{.Size}}')
  echo "  OK: ${AUTOWARE_IMAGE} (${SIZE})"
else
  echo "  FAIL: ${AUTOWARE_IMAGE} が見つかりません"
  exit 1
fi

# 5. AWSIM + サンプルマップ確認
AWSIM_DIR="${AWSIM_DIR:-/workspace/ai_coding_ws/awsim}"
echo "[5/5] AWSIM + サンプルマップ..."
AWSIM_BIN="${AWSIM_BIN:-${AWSIM_DIR}/awsim_labs_v1.6.1/awsim_labs.x86_64}"
MAP_DIR="${AWSIM_MAP_PATH:-${AWSIM_DIR}/sample_map/nishishinjuku_autoware_map}"

if [ -x "${AWSIM_BIN}" ]; then
  echo "  AWSIM binary: OK"
else
  echo "  FAIL: ${AWSIM_BIN} が見つからないか実行権限がありません"
  exit 1
fi

if [ -f "${MAP_DIR}/pointcloud_map.pcd" ] && [ -f "${MAP_DIR}/lanelet2_map.osm" ]; then
  echo "  Sample map: OK (PCD + OSM)"
else
  echo "  FAIL: サンプルマップが不完全です"
  exit 1
fi

echo ""
echo "=== 全チェック OK ==="
echo ""
echo "次のステップ:"
echo "  ターミナル1: bash scripts/run_awsim_autoware_demo.sh awsim"
echo "  ターミナル2: bash scripts/run_awsim_autoware_demo.sh autoware"
echo "  ターミナル3: bash scripts/run_awsim_autoware_demo.sh engage"
