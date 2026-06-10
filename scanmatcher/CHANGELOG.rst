^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scanmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
