# Independent first-map validation ledger

Status: **0 / 3 accepted validations**.

This ledger is the public evidence source for the v1.0 external-adoption gate.
Maintainer-operated Docker, source, CI and real-data runs do not count here.
They establish the test boundary but cannot substitute for independent
onboarding experience.

## Submit a validation

Open an
[Independent First-Map Validation](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=first-map-validation.yml)
issue and record the first attempt before receiving private maintainer setup
instructions. Successful and failed attempts are both requested.

Do not publish private bags, credentials, exact site coordinates, faces,
license plates or other sensitive data. Redacted manifests, verifier summaries,
diagnosis output and relevant log excerpts are sufficient for triage.

## Acceptance criteria

A report counts toward the gate only when all of these are true:

1. The tester is not the maintainer and did not author the tested change.
2. The attempt begins from public repository documentation without private
   setup instructions.
3. It uses one of the three entrypoints in
   [`first-map-v1.json`](../contracts/first-map-v1.json).
4. The report identifies an immutable commit, release tag or image digest and
   records the environment and exact commands.
5. A completed run contains the eight required success artifacts, and map
   verification reports PASS.
6. First-attempt onboarding failures are recorded and each release-blocking
   finding is resolved or explicitly documented before v1.0 sign-off.
7. A maintainer reviews the evidence and adds the accepted report to this
   ledger and its machine-readable
   [JSON companion](independent-first-map-validations.json).

Multiple reports from the same person, machine or materially identical
environment count once. Maintainer-assisted reruns may close a finding but do
not erase the first-attempt result.

## Accepted validations

| ID | Issue | Entrypoint | Environment | Tested identity | First attempt | Accepted evidence |
| --- | --- | --- | --- | --- | --- | --- |
| _No accepted validations yet_ | — | — | — | — | — | — |

## Open findings

No independent reports have been submitted yet. This section must link every
unresolved onboarding finding once reports arrive.
