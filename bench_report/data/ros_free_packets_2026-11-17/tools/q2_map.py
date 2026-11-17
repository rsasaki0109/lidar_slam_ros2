"""Q2: python -m lidarslam's mapping with the parameters lidarslam-map recorded (core_params.json)."""
import json, sys
from lidarslam import mapping
params = json.load(open(sys.argv[2]))
summary = mapping.run(sys.argv[1], params['odometry'], params['backend'], sys.argv[3])
print(json.dumps(summary, default=str))
