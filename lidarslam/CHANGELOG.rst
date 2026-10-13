^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lidarslam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.0 (2026-10-13)
-------------------
* Ship the ENWIDE v10 radar-less Ouster options as the
  ``photometric_bump_no_radar`` preset.
* Add the handheld MID-360 outdoor RKO-LIO profile (scan gaps up to 3 s,
  gravity re-levelling, occluded-scan skipping) and an RKO-LIO online odometry
  launch with MID-360 defaults that converts IMU acceleration from g.
* Deskew the NTU VIRAL profile with the interval's mean angular velocity again;
  RKO-LIO's samplewise gyro deskew had raised the tnp_01 APE from 0.936 to
  1.14 m.
* Make RKO-LIO wait for its own output subscribers, level MID-360 preset maps
  with RKO-LIO's gravity initialization, and report map frames tilted from
  gravity in diagnosis.
* Contributors: Ryohei Sasaki

0.9.1 (2026-08-12)
------------------
* Add one interactive product home, bag-optional system doctor, guided sensor
  review, session history, comparison, support, and first-map sharing paths.
* Unify Docker and source first-map demos on the fixed MID-360 profile,
  verifier, diagnosis, manifest, and privacy-bounded receipt contract.
* Add the exact six-package Humble/Jazzy source bootstrap and installed-product
  checks while keeping package-manager instructions evidence-gated.
* Bind canonical first-map documentation bytes to the exact Pages deployment
  revision before an independent-user cohort can start.
* Derive clean-host release packets from one audited report and recheck the
  exact source commit and Humble/Jazzy image digests before measurement.
* Contributors: Ryohei Sasaki

0.9.0 (2026-07-30)
------------------
* Add the fail-closed v1 readiness audit and preserve the three-entrypoint
  installed product contract.
* Require official ``rko_lio >= 0.3.2`` after its Humble/Jazzy installed
  golden-path E2E passed.
* Add schema-backed ROS apt clean-install and main-to-testing upgrade
  evidence, including installed CLI, stale-path, and pinned real-map gates.
* Keep the beginner apt path disabled until normal-repository evidence exists.
* Contributors: Ryohei Sasaki

0.7.0 (2026-07-29)
------------------
* Add the installed ``lidarslam-map`` product CLI with doctor, run, inspect,
  optional view, completion, and machine-readable option contracts.
* Normalize path and duration metavars, reject non-positive viewer durations,
  and enforce exact option/completion parity plus stable usage exit codes.
* Keep doctor and runner profile options synchronized through one installed
  maintained-profile registry.
* Add preflight-v3, diagnosis-v1, resumable run-manifest-v2, atomic output,
  collision and storage refusal, termination recovery, and digest-only
  rollback planning.
* Emit a schema-validated, privacy-bounded first-map validation receipt from
  the finalized manifest, diagnosis, and canonical Autoware verifier result.
* Validate clean-prefix Humble/Jazzy installation and v0.6.0 upgrade parity.
* Contributors: Ryohei Sasaki

0.6.0 (2026-06-12)
------------------
* Presets updated for the v0.6 deterministic backend: event-driven loop
  search is the default and ``deterministic_loop_scheduling`` is retired.
* Contributors: Ryohei Sasaki

0.5.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* SPDX license tag (BSD-2-Clause).
* Launch files for the public workflows: ``lidarslam.launch.py`` (scanmatcher
  frontend) and ``rko_lio_slam.launch.py`` (RKO-LIO frontend; requires the
  ``rko_lio`` package from a source workspace until it is released to
  rosdistro).
* Dataset parameter presets (NTU VIRAL, Livox MID-360, RTK-SLAM, KITTI, and
  others) maintained under ``lidarslam/param/``.
* Contributors: Ryohei Sasaki
