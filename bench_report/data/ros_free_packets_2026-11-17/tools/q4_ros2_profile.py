"""Q4: ROS 2 engine on Leo Drive bag1: automatic profile, and the forced RKO-LIO profile."""
import importlib.util, sys
from pathlib import Path
repo = Path(sys.argv[1]); bag = Path(sys.argv[2])
spec = importlib.util.spec_from_file_location('runner', repo / 'scripts/run_autoware_map_from_bag.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
pre = m._load_script_module('preflight_autoware_map_bag.py', 'preflight_autoware_map_bag')
payload = pre.build_preflight_payload(bag)
print('recommendations:', [(r['id'], r['priority'], r.get('engines')) for r in payload['recommendations']])
for engine in ('ros2', 'core'):
    plan = m.build_execution_plan(bag, None, Path('/nonexistent/out'), True, preflight_payload=payload, engine=engine)
    print(f'{engine} auto profile:', plan['profile_id'])
try:
    m.build_execution_plan(bag, 'rko_lio_graph_public_path', Path('/nonexistent/out'), True, preflight_payload=payload, engine='ros2')
    print('forced ros2: NOT refused')
except ValueError as exc:
    print('forced ros2 refused:', exc)
