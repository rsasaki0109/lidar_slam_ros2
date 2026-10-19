#!/usr/bin/env bash
# flat-room trial-1 and underground-garage trial-3 frontend-only variants (scale-aware scoring), then Oxford.
# Usage: mapshd_two_trials.sh "<overrides>" ...
set -uo pipefail
SWEEP=front_sweep_scale.sh OUT=/home/user/bench/scale/mapshd2 PARAMS=/home/user/bench/params/mapshd_gyro002_jerk1000.yaml \
  SCENARIOS=flat-room bash /home/user/scripts/mapshd_front_sweep.sh /home/user/bench/scale/sweep_mapshd2.csv "$@"
SWEEP=front_sweep_scale.sh OUT=/home/user/bench/scale/mapshd2 PARAMS=/home/user/bench/params/mapshd_gyro002_jerk1000.yaml \
  SCENARIOS=underground-garage TRIALS=trial-3 bash /home/user/scripts/mapshd_front_sweep.sh /home/user/bench/scale/sweep_mapshd2.csv "$@"
YAML_SUFFIX=_jerk1000 bash /home/user/scripts/front_sweep_other.sh /home/user/bench/scale/sweep_other2.csv /home/user/bench/scale/other2 oxford "$@"
echo "TWO TRIALS DONE $(date -u +%T)"
