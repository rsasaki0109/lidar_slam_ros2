^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lidarslam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
