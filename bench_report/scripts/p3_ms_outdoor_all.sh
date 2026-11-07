#!/bin/bash
# All runs of plan.txt (candidate 1 and amendment 1's candidate 2), after the as-is con/Stadtgarten runs.
set -uo pipefail
B=/home/user/bench/ms_outdoor; R=/home/user/lidar_slam_ros2/bench_report/scripts/p3_ms_outdoor_run.sh; E=$(cat $B/con-1/extrinsic.txt); ID=0.0,0.0,0.0,1.0,0.0,0.0,0.0
MH=0.0,0.0,0.0,1.0,-0.011,-0.02329,0.04412; EX=0.7071068,-0.7071068,0.0,0.0,-0.001,-0.00855,0.055
row() { python3 - "$1" <<'PY'
import sys, yaml
r = yaml.safe_load(open(sys.argv[1]))
print(','.join(repr(float(v)) for v in next(s for s in r['sessions'] if s.get('session') == 1)['transform_to_session_0']))
PY
}
ms() { python3 -c "import json; print(','.join(repr(v) for v in json.load(open('/home/user/bench/ms_rtk/$1'))['debug_exclude_transform']))"; }
XC12=$(row $B/runs/conA12/join_report.yaml); XC21=$(row $B/runs/conA21/join_report.yaml)
XS12=$(ms alone_A1_B2.json); XS21=$(ms alone_A2_B1.json)
set_runs() {  # set_runs PREFIX REF
  local p=$1 ref=$2
  bash $R pair ${p}_con12 $ref con-1 con-2 $E; bash $R pair ${p}_con21 $ref con-2 con-1 $E
  bash $R pair ${p}_conx12 $ref con-1 con-2 $E $XC12; bash $R pair ${p}_conx21 $ref con-2 con-1 $E $XC21
  bash $R pair ${p}_sg12 $ref sg1 sg2 $ID; bash $R pair ${p}_sg21 $ref sg2 sg1 $ID
  bash $R pair ${p}_sgx12 $ref sg1 sg2 $ID $XS12; bash $R pair ${p}_sgx21 $ref sg2 sg1 $ID $XS21
  bash $R split ${p}_calib $ref calib1 $MH; bash $R split ${p}_calibx $ref calib1 $MH stress
  bash $R split ${p}_exp04 $ref exp04 $EX; bash $R split ${p}_exp04x $ref exp04 $EX stress
}
# as is (bar 0.5, candidate 1's consistency reported): the runs not yet made as is
bash $R pair conAx12 1e-9 con-1 con-2 $E $XC12; bash $R pair conAx21 1e-9 con-2 con-1 $E $XC21
bash $R split A_calib 1e-9 calib1 $MH; bash $R split A_calibx 1e-9 calib1 $MH stress
bash $R split A_exp04 1e-9 exp04 $EX; bash $R split A_exp04x 1e-9 exp04 $EX stress
# candidate 1: c0 = min(c of con-1, con-2) from conA12
C1=$(python3 - <<'PY'
import yaml
s = next(s for s in yaml.safe_load(open('/home/user/bench/ms_outdoor/runs/conA12/join_report.yaml'))['sessions'] if s.get('session') == 1)
print(repr(min(s['consistency']['a'], s['consistency']['b'])))
PY
)
echo "candidate 1 c0=$C1"
set_runs c1 $C1
# candidate 2: revisits only; c0 from an as-is run that reports c_rev of the con pair
SKIP=10 bash $R pair conR12 1e-9 con-1 con-2 $E
C2=$(python3 - <<'PY'
import yaml
s = next(s for s in yaml.safe_load(open('/home/user/bench/ms_outdoor/runs/conR12/join_report.yaml'))['sessions'] if s.get('session') == 1)
print(repr(min(s['consistency']['a'], s['consistency']['b'])))
PY
)
echo "candidate 2 c0=$C2"
export SKIP=10; set_runs c2 $C2
echo all done
