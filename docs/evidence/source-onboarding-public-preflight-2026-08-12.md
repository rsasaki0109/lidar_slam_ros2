# Source onboarding public preflight — 2026-08-12

> Status: **LOCAL_CONTRACT_PASS / PUBLIC_CANDIDATE_NOT_READY**
>
> Remote mutations performed: **none**

## Finding

The source quickstart said it built six repository packages, but its build used
an unrestricted repository discovery result. It did not compare the discovered
names with the maintained set or pass an explicit selection to the build. An
accidentally added experimental package could therefore expand beginner build
time and dependency failures while the plan still promised six packages.

The disposable-host observer also checked only that a public commit contained
the quickstart invocation, Getting Started invocation, and matching `VERSION`.
It did not prove that the public helper retained the fast tests-disabled build,
repository-only dependency route, or exact package scope. The manual runbook
made a stricter check, but searched Getting Started for
`-DBUILD_TESTING=OFF` while the page intentionally documents
`BUILD_TESTING=OFF`; that copied preflight would reject the current contract.

## Repair

`source_quickstart.sh` now owns one ordered package inventory:

```text
graph_based_slam
lidarslam
lidarslam_msgs
ndt_omp_ros2
rko_lio
scanmatcher
```

After pinned submodules and build tools are available, but before rosdep or
compilation, the helper runs `colcon list --base-paths ... --names-only`, sorts
the result, and requires exact equality. It then passes the same ordered array
to `colcon build --packages-select`. Missing or extra packages fail with
`[source-package-inventory-mismatch]`; dependency resolution, build, and demo
are not started. The source observer recognizes that private log marker and
stores the same stable finding in its privacy-bounded trial record.

The observer also provides:

```bash
python3 scripts/run_source_onboarding_probe.py \
  --public-preflight \
  --source-commit <40-lowercase-hex-commit> \
  --product-version 0.9.0
```

This mode requires no trial VM, ROS installation, output path, privilege
acknowledgement, or active-time choice. It reads the public GitHub API and
returns JSON only:

- `READY`, exit `0`: the exact commit and complete route contract pass;
- `NOT_READY`, exit `1`: the immutable route is absent or incomplete; or
- exit `2`: API, decoding, or observer failure, which must not be translated
  into a false absence claim.

For one immutable commit, it checks commit identity, the exact shell package
array, explicit package selection and mismatch marker, repository-only rosdep
helper, tests-disabled build, demo delegation, canonical Getting Started
instructions, and `VERSION`. It sets `sys.dont_write_bytecode` before importing
adjacent helpers; public preflight therefore does not create cache files in the
observer checkout.

## Actual public check

The local cumulative candidate is based on public commit
`3f4dd70cdc58ad421192559213cdee0bdc41eba8` but is intentionally dirty and has
not been published as a new immutable commit. Running public preflight for
version `0.9.0` against that base returned:

```text
status=NOT_READY
exit=1
finding=source-route-contract-missing
detail=public commit lacks scripts/source_quickstart.sh
writes_performed=false
```

This is the expected fail-closed result. The observer did not substitute the
private checkout, a local clone URL, `develop`, or a moving tag. A before/after
snapshot of all `scripts/**/__pycache__`, `.pyc`, and `.pyo` paths, sizes, and
mtimes remained identical.

## Verification

| Check | Result |
| --- | --- |
| exact six-package plan and explicit selection, Jazzy host | PASS; no quickstart writes |
| same plan, fixed Humble image, source read-only, network disabled | PASS; no quickstart writes |
| extra-package failure before dependency/build | PASS |
| stable mismatch finding across private-log boundary | PASS |
| public route contract, READY/NOT_READY, parser and read-only regressions | PASS |
| focused source quickstart/observer regressions | `26 passed` |
| combined docs/source regressions | `41 passed` |
| changed Python `ament_flake8` | PASS |
| shell syntax | PASS |
| MkDocs strict build | PASS with existing Material/nav notices |
| canonical unsourced Python gate | graph `1,428 passed, 13 skipped, 11 warnings`; lidarslam `631 passed`; `2,059` total |

The final cumulative milestone receipt binds this gate and the patch identity.

## Limits and next gate

This proves selection, preflight semantics, and observer safety. It does not
publish the private candidate, install dependencies on a clean VM, download the
fixed 517 MB bag, produce a map, or create a comparable onboarding row.

After a separately authorized reviewed commit is public, run this preflight
first. Only a `READY` result authorizes provisioning the clean Humble and Jazzy
source-trial VMs. The measured route must then repeat preflight and satisfy the
existing wall-time, active-time, command-count, download, peak-disk, output,
verifier, and receipt contract.

No commit, branch, pull request, issue, label, release, image, package, review
reply, or external repository was changed.
