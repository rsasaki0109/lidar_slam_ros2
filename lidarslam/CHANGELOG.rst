^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lidarslam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
