# Support

`lidarslam_ros2` is a maintainer-led community project. Support is best effort;
there is no guaranteed response or resolution time.

## Start here

1. Read the [Product Contract](docs/product-contract.md).
2. Follow [Getting Started](docs/getting-started.md).
3. Run:

```bash
bash scripts/run_autoware_map_beginner.sh <rosbag2_dir> --preflight-only
```

4. Search existing issues before opening a new one.

## Where to report

- Reproducible product defect: use the **Bug report** issue form.
- New sensor or rig: use the **Sensor support** issue form.
- Proposed behavior: use the **Feature request** issue form.
- Benchmark result: use the **Benchmark report** issue form.
- Autoware map bundle problem: use the **Autoware pointcloud map** issue form.
- Vulnerability: follow [SECURITY.md](SECURITY.md), not a public issue.
- Conduct incident: follow [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## Required diagnostic bundle

Reports are actionable when they include:

- exact commit, release tag or container digest;
- ROS distribution, Ubuntu version and architecture;
- exact command and parameter files;
- sensor model, topics and frame names;
- preflight output;
- `autoware_map_diagnosis.md` and `verify_autoware_map.log`;
- relevant logs and the smallest shareable bag or metadata reproduction;
- expected and observed behavior.

Do not upload private location data, credentials or proprietary bags without
authorization.

## Support boundaries

Validated public demos and maintained-compatible product paths receive
priority. Research scripts, custom hardware, Windows, modified forks and
third-party optional algorithms may receive guidance but are not guaranteed
integration support. See the support tiers in
[the Product Contract](docs/product-contract.md).

Questions that cannot be reproduced may be converted into documentation
requests or closed with a request for the missing diagnostic bundle.
