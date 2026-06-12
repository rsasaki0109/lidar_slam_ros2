^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scanmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-12)
------------------
* Deterministic core / ROS shell refactor (v0.6 roadmap): pose prediction,
  pose acceptance (including the Tracking/Suspect/Recovery state machine),
  IMU orientation processing and the map-update/local-map policy moved into
  tested pure headers; the component shell keeps registration and pub/sub
  I/O only.
* New ``scan_matcher_offline_runner``: drives the component in lockstep over
  a raw sensor bag (intra-process pub/sub, drained single-threaded executor,
  synchronous map update); three runs over the full NTU VIRAL tnp_01 bag are
  byte-identical — the first determinism coverage for this frontend.
* Contributors: Ryohei Sasaki

0.5.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* SPDX license tag (BSD-2-Clause).
* NDT (ndt_omp_ros2) scan-matching frontend; FastGICP / SmallGICP registration
  backends are enabled automatically when those optional packages are present
  at build time.
* IMU pre-integration and initial-pose options on the classic
  scanmatcher-frontend workflow.
* Contributors: Ryohei Sasaki
